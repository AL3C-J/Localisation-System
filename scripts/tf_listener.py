#!/usr/bin/env python3
import rospy
import tf2_ros
from std_msgs.msg import Float32

def main():
    rospy.init_node('tf_chain_publisher_node')

    # Create a tf2_ros Buffer and Listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Publishers for x, y, z components
    pub_x = rospy.Publisher('/map_to_base_link_x', Float32, queue_size=10)
    pub_y = rospy.Publisher('/map_to_base_link_y', Float32, queue_size=10)
    pub_z = rospy.Publisher('/map_to_base_link_z', Float32, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            # Get the transform from 'map' to 'base_link'
            transform = tf_buffer.lookup_transform('camera_frame', 'base_link', rospy.Time(0))

            # Extract translation components (x, y, z)
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            # Publish the x, y, z components
            pub_x.publish(x)
            pub_y.publish(y)
            pub_z.publish(z)

            # Optionally log the values for debugging
            rospy.loginfo(f"Published Translation: x={x}, y={y}, z={z}")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform error: {e}")

        rate.sleep()

if __name__ == '__main__':
    main()
