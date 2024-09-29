#!/usr/bin/env python

import rospy
from duckietown_msgs.msg import WheelCmd
import time

def drive_straight():
    # Initialize the ROS node
    rospy.init_node('drive_straight_node', anonymous=True)
    
    # Create a publisher for the car command
    pub = rospy.Publisher('/car_cmd_switch_node/car_cmd', WheelCmd, queue_size=10)
    
    # Set the rate of publishing
    rate = rospy.Rate(10)  # 10 Hz

    # Create a WheelCmd message
    cmd = WheelCmd()
    cmd.vel_left = 0.3  # Adjust speed as necessary (m/s)
    cmd.vel_right = 0.3  # Adjust speed as necessary (m/s)

    # Duration to drive straight (~1m distance)
    distance = 1.0  # in meters
    time_to_drive = distance / cmd.vel_left  # time = distance / speed

    # Publish the command for the calculated time
    start_time = time.time()
    while not rospy.is_shutdown() and (time.time() - start_time < time_to_drive):
        pub.publish(cmd)
        rate.sleep()

    # Stop the robot
    cmd.vel_left = 0.0
    cmd.vel_right = 0.0
    pub.publish(cmd)

if __name__ == '__main__':
    try:
        drive_straight()
    except rospy.ROSInterruptException:
        pass

