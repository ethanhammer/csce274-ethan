#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

def senderer():
   
    pub = rospy.Publisher('delta', Float32, queue_size=10)
    
    rospy.init_node('publisher', anonymous=True)
    
    rate = rospy.Rate(30)  
    
    while not rospy.is_shutdown():
    
        msg = Float32()
        msg.data = 10.0  
        
        pub.publish(msg)  
        
        rate.sleep()

if __name__ == '__main__':
    try:
        senderer()
    except rospy.ROSInterruptException:
        pass

