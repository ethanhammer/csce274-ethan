#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32  # Import Float32 instead of Int32

def callback(msg):
    # This function will be called whenever a new message is received
    rospy.loginfo("Received: %f", msg.data)  # Log message data as a float

def listener():
    # Initialize the ROS node
    rospy.init_node('subscriber_node', anonymous=True)
    
    # Create a subscriber to the "total" topic with std_msgs/Float32 messages
    rospy.Subscriber('total', Float32, callback)
    
    # Keep the node running to process incoming messages
    rospy.spin()

if __name__ == '__main__':
    listener()

