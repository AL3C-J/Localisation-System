#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations
import numpy as np

def pose_callback(msg):
        # Extract rotation matrix from Quaternion
        quaternion = (msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)
        rotation_matrix = tf.transformations.quaternion_matrix(quaternion)
        #print(rotation_matrix[0][3], rotation_matrix[1][3], rotation_matrix[2][3])

        # Offset vector in the marker's local coordinate frame
        offset_vector = [0.289, 0.0, 0.0, 1.0]  # 28.9 cm along the x-axis of the marker
        # Calculate the non-static directional offset vector (rotation matrix x offset vector)
        transformed_offset = rotation_matrix.dot(offset_vector)
        # Add the transformed offset to the marker's position to get the rover's center position
        rover_center_x = msg.pose.position.x + transformed_offset[0]
        rover_center_y = msg.pose.position.y + transformed_offset[1]
        rover_center_z = msg.pose.position.z + transformed_offset[2]
        # Log the rover's center position (TOGGLE FOR DEBUGGING)
        rospy.loginfo(f"Rover Center Pose: Position -> x: {rover_center_x}, y: {rover_center_y}, z: {rover_center_z}")

        # ---------------- Set up msg for publishing -----------------#
        rover_pose = PoseStamped()
        rover_pose.header = msg.header  # Keep the same timestamp and frame
        rover_pose.pose.position.x = rover_center_x
        rover_pose.pose.position.y = rover_center_y
        rover_pose.pose.position.z = rover_center_z
        rover_pose.pose.orientation = msg.pose.orientation  # Orientation remains the same
        # Publish the transformed pose
        rover_pose_pub.publish(rover_pose)
        #-------------------------------------------------------------#
        
def main():
    rospy.init_node('marker_transformer_node')
    # PUBLISH Rover Pose
    global rover_pose_pub
    rover_pose_pub = rospy.Publisher('/rover_pose', PoseStamped, queue_size=10)
    # SUBSCRIBE to Marker Pose
    rospy.Subscriber('/marker_pose', PoseStamped, pose_callback)
    # Keep the node running to listen for incoming messages
    rospy.spin()

if __name__ == '__main__':
    main()
