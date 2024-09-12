#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32  

def talker():
    
    pub = rospy.Publisher('delta', Int32, queue_size=10)
    
    # Initialize the ROS node
    rospy.init_node('publisher', anonymous=True)
    
    rate = rospy.Rate(10)  # 10hz
    
    while not rospy.is_shutdown():

        msg = Int32()
        msg.data = 10  # Set the message data
        
        pub.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

