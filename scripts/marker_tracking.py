#!/usr/bin/env python3
import numpy as np
import cv2
import cv2.aruco as aruco
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

#-------------- Parameters -----------------#
camera_matrix = np.array([
    [1764.9,    0,     1010.9],  
    [   0,     1769.5, 513.4],  
    [   0,        0,    0.001]   
])#Iphone8+ Camera callibration matrices
dist_coeffs = np.array([0.2929, -1.1367, 0, 0, 0])
# (ID : Size in meters)
marker_sizes = {
    5: 0.081  } 
#-------------------------------------------#

# Global variable to store the latest rover position
rover_position = None

def rover_pose_callback(msg):
    global rover_position
    # Store the received rover position for use in the main loop
    rover_position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

def main():
    rospy.init_node("node_marker_tracking")
    
    # Load the video file
    video_capture = cv2.VideoCapture('/home/localise_rover/catkin_ws/src/sub_marker_tracking/videos/Board_Test_1.MOV')
    
    # Create a dictionary of markers (retrieve information for marker recognition)
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()
    
    #--------------------------SET UP PUBLISHER----------------------------#
    marker_pose_pub = rospy.Publisher('/marker_pose', PoseStamped, queue_size=10)
    
    #--------------------------SET UP SUBSCRIBER----------------------------#
    rospy.Subscriber('/rover_pose', PoseStamped, rover_pose_callback)
    
    rate = rospy.Rate(10)  # 10 Hz

    # ======================= Enter Main loop =============================#
    while not rospy.is_shutdown():
        ret, frame = video_capture.read()
        if not ret:
            rospy.loginfo("End of Video Clip")
            break

        # Convert frames to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # DETECT
        corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        
        # Calculate rvecs and tvecs for Marker 5
        if corners is not None and ids is not None:
            detected_ids = ids.flatten()
            if 5 in detected_ids:
                i = np.where(detected_ids == 5)[0][0]
                marker_size = marker_sizes[5]
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, dist_coeffs)
                frame_markers = cv2.drawFrameAxes(frame_markers, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], 0.1)
                x, y, z = tvecs[0][0]
                 # Log the rover's center position (TOGGLE FOR DEBUGGING)
                #rospy.loginfo(f"DEBUGGING MARKER POS -> x: {x}, y: {y}, z: {z}")
                x_rvec, y_rvec, z_rvec = rvecs[0][0]
                x_deg, y_deg, z_deg = np.degrees([x_rvec, y_rvec, z_rvec])

                #------------------PUBLISH POSE INFORMATION (x, y, z, qx, qy, qz, qw)----------------------#
                qx, qy, qz, qw = quaternion_from_euler(float(x_rvec), float(y_rvec), float(z_rvec))
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "camera_frame"

                pose_msg.pose.position.x = float(x)
                pose_msg.pose.position.y = float(y)
                pose_msg.pose.position.z = float(z)
                
                pose_msg.pose.orientation.x = qx
                pose_msg.pose.orientation.y = qy
                pose_msg.pose.orientation.z = qz
                pose_msg.pose.orientation.w = qw

                marker_pose_pub.publish(pose_msg)

                #----------------------------------Display 6-States on Screen------------------------------#
            position_text = f"Rover Pos (x,y,z): {x:.2f}, {y:.2f}, {z:.2f} meters"
            rotation_text = f"Rover Rot (x,y,z): {x_deg:.2f}, {y_deg:.2f}, {z_deg:.2f} degrees"
            font_scale = 1  
            font_thickness = 2  
            max_text_width = max(cv2.getTextSize(position_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)[0][0],
                                cv2.getTextSize(rotation_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)[0][0])
            total_text_height = cv2.getTextSize(position_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)[0][1] * 2 + 40
            cv2.rectangle(frame_markers, (10, 30), (10 + max_text_width + 20, 30 + total_text_height), (255, 255, 255), cv2.FILLED)
            cv2.putText(frame_markers, position_text, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 255), font_thickness, cv2.LINE_AA)
            cv2.putText(frame_markers, rotation_text, (20, 100), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 255), font_thickness, cv2.LINE_AA)
        
            # -------------------- Draw Rover Center Dot -------------------- #
            #point_3d = np.array([rover_position])
            # Project the 3D point to 2D
            #point_2d, _ = cv2.projectPoints(point_3d, np.zeros((3, 1)), np.zeros((3, 1)), camera_matrix, dist_coeffs)
            # Print the projected point [TOGGLE for DEBUGGING]
            #print("Projected 2D point:", point_2d)
            # Draw the point on the image
            #pixel_coordinates = tuple(int(c) for c in point_2d.ravel())
            #cv2.circle(frame_markers, pixel_coordinates, 10, (0, 255, 0), -1)
            #cv2.circle(frame_markers, np.array([point_3d[0], point_3d[1]]), 10, (0, 255, 0), -1)
            # --------------------------------------------------------------- #

        cv2.imshow('Frame with Pose Estimation', frame_markers)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    video_capture.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
