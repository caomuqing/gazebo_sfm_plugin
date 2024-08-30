#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

def publish_message():
    # Initialize the ROS node
    rospy.init_node('float32_array_publisher', anonymous=True)
    
    # Create a publisher object
    pub = rospy.Publisher('/actor4/test', Float32MultiArray, queue_size=10)
    
    # Define the message content
    msg = Float32MultiArray()
    msg.data = [3.0, 0.0, 3.0, -3.0]

    rospy.loginfo("Press Enter to send the message...")
    
    while not rospy.is_shutdown():
        # Wait for user input (Enter key press)
        input()
        
        # Publish the message
        pub.publish(msg)
        # rospy.loginfo("Message sent: [0, 0, 3, 3]")

        rospy.sleep(1)  # Sleep to avoid multiple messages being sent too quickly

if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
