#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32  

def callback(msg):
    rospy.loginfo("Received: %d", msg.data)

def listenerer():
    
    rospy.init_node('subscriber_node', anonymous=True)
    
    rospy.Subscriber('total', Float32, callback)
    
    rospy.spin()

if __name__ == '__main__':
    listenerer()

