#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32  # Import Float32 instead of Int32

def talker():
    # Create a publisher for Float32 messages
    pub = rospy.Publisher('delta', Float32, queue_size=10)
    
    # Initialize the ROS node
    rospy.init_node('publisher', anonymous=True)
    
    rate = rospy.Rate(10)  # 10hz
    
    while not rospy.is_shutdown():
        msg = Float32()  # Create a Float32 message
        msg.data = 10.0  # Set the message data as a float
        
        pub.publish(msg)  # Publish the message
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

