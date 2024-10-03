#!/usr/bin/env python3
import numpy as np
import cv2
import cv2.aruco as aruco
import rospy
from tf.transformations import quaternion_from_euler
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

#-------------- Parameters -----------------#
#Iphone8+ Camera:
#camera_matrix = np.array([
#    [1764.9,    0,     1010.9],  
#    [   0,     1769.5, 513.4],  
#    [   0,        0,    0.001]   
#])
#dist_coeffs = np.array([0.2929, -1.1367, 0, 0, 0])
#RealSense:
camera_matrix = np.array([
    [1375.5,    0,     977.4],  
    [   0,     1376.7, 555.5],  
    [   0,        0,    1]   
])
dist_coeffs = np.array([0.1196, -0.2006, 0, 0, 0])

# (ID : Size in meters)
marker_sizes = {
    5: 0.081  } 
#-------------------------------------------#
def rotation_matrix_to_euler_angles(R):
    """
    Converts a rotation matrix to Euler angles (in radians).
    Assumes the rotation matrix is for rotations about the x, y, and z axes in the order z-y-x (intrinsic rotations).
    Returns:(roll, pitch, yaw)
    """
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

# Global variable to store the latest rover position
rover_position = None
video_capture = None

def rover_pose_callback(msg):
    global rover_position
    # Store the received rover position for use in the main loop
    rover_position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

def callback(data):
    global video_capture
    bridge = CvBridge()  
    try:
      video_capture = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

def main():
    
    rospy.init_node("node_marker_tracking")  
    
    global video_capture
    
    # Create a dictionary of markers (retrieve information for marker recognition)
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()
     
    rate = rospy.Rate(30)  # 30 Hz

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer) #dont remove
   
    marker_pose_pub = rospy.Publisher('/rover_centre', PoseStamped, queue_size=10) #Pose publisher

    rospy.Subscriber("/camera/color/image_raw",Image,callback)
    rospy.sleep(3)
    # ======================= Enter Main loop =============================#
    while not rospy.is_shutdown():
        frame = video_capture
        # Print the resolution of the original frame
        print(video_capture.shape)

        # Convert frames to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # DETECT
        corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        
        x_base_link = 0
        y_base_link = 0
        z_base_link = 0

        # Calculate rvecs and tvecs for Marker 5
        if corners is not None and ids is not None:
            detected_ids = ids.flatten()
            if 5 in detected_ids:
                i = np.where(detected_ids == 5)[0][0]
                marker_size = marker_sizes[5]
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, dist_coeffs)
                frame_markers = cv2.drawFrameAxes(frame_markers, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], 0.1)
                x, y, z = tvecs[0][0]

                rotation_matrix, _ = cv2.Rodrigues(rvecs)
                # Optional: Convert rotation matrix to Euler angles
                euler_angles = rotation_matrix_to_euler_angles(rotation_matrix)
                #euler_angles_deg = np.degrees(euler_angles)               

                #print("Euler Angles (degrees):", euler_angles_deg)

                x_rvec, y_rvec, z_rvec = euler_angles[0], euler_angles[1], euler_angles[2]
                                      
                q = quaternion_from_euler(float(x_rvec), float(y_rvec), float(z_rvec))
                qx = q[0]
                qy = q[1]
                qz = q[2]
                qw = q[3]

                #-------------BROADCAST TRAN-------------#
                br = tf2_ros.TransformBroadcaster()
                t = geometry_msgs.msg.TransformStamped()

                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "camera_frame"
                t.child_frame_id = "marker_frame"
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = z
                t.transform.rotation.x = qx
                t.transform.rotation.y = qy
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw
                br.sendTransform(t)
                
                try:
                    # Get the transform from 'camera' to 'base_link'                  
                    transform = tf_buffer.lookup_transform('camera_frame', 'base_link', rospy.Time(0))
                    # Extract translation components (x, y, z)
                    x_base_link = transform.transform.translation.x
                    y_base_link = transform.transform.translation.y
                    z_base_link = transform.transform.translation.z

                    # Publish the marker pose
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = rospy.Time.now()
                    pose_msg.header.frame_id = "base_link"
                    pose_msg.pose.position.x = x_base_link
                    pose_msg.pose.position.y = y_base_link
                    pose_msg.pose.position.z = z_base_link
                    
                    pose_msg.pose.orientation.x = transform.transform.rotation.x
                    pose_msg.pose.orientation.y = transform.transform.rotation.y
                    pose_msg.pose.orientation.z = transform.transform.rotation.z
                    pose_msg.pose.orientation.w = transform.transform.rotation.w
                    
                    marker_pose_pub.publish(pose_msg)

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn(f"Transform error: {e}")



                #----------------------------------Display 6-States on Screen------------------------------#
            marker_centre = f"Marker Pos (x,y,z): {x:.2f}, {y:.2f}, {z:.2f} meters"
            rover_centre = f"Rover Pos (x,y,z): {x_base_link:.2f}, {y_base_link:.2f}, {z_base_link:.2f} meters"
            font_scale = 1  
            font_thickness = 2  
            max_text_width = max(cv2.getTextSize(marker_centre, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)[0][0],
                                cv2.getTextSize(rover_centre, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)[0][0])
            total_text_height = cv2.getTextSize(marker_centre, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)[0][1] * 2 + 40
            cv2.rectangle(frame_markers, (10, 30), (10 + max_text_width + 20, 30 + total_text_height), (255, 255, 255), cv2.FILLED)
            cv2.putText(frame_markers, marker_centre, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 255), font_thickness, cv2.LINE_AA)
            cv2.putText(frame_markers, rover_centre, (20, 100), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 255), font_thickness, cv2.LINE_AA)
        
            # -------------------- Draw Rover Center Dot -------------------- #           
            point_3d = np.array([[x_base_link, y_base_link, z_base_link]], dtype=np.float64)                       
            # Project the 3D point to 2D
            point_2d, _ = cv2.projectPoints(point_3d, np.zeros((3, 1)), np.zeros((3, 1)), camera_matrix, dist_coeffs)                          
            # Draw the point on the image
            pixel_coordinates = tuple(int(c) for c in point_2d.ravel())
            cv2.circle(frame_markers, pixel_coordinates, 10, (0, 255, 0), -1)           
            # --------------------------------------------------------------- #

        cv2.imshow('Frame with Pose Estimation', frame_markers)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass