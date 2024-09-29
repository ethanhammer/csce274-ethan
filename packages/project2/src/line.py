#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_duckiebot():
    rospy.init_node('line', anonymous=True)
    pub = rospy.Publisher('/bigbot/car_cmd_switch_node/cmd', Twist, queue_size=10)

    # Parameters
    distance = 1.0  # 1 meter
    speed = 0.3     # m/s
    time_to_move = distance / speed  # time in seconds

    # Create a Twist message for forward movement
    move_command = Twist()
    move_command.linear.x = speed
    move_command.linear.y = 0
    move_command.linear.z = 0
    move_command.angular.x = 0
    move_command.angular.y = 0
    move_command.angular.z = 0

    # Publish the move command
    rospy.loginfo("Moving Duckiebot forward...")
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.get_time()

    while rospy.get_time() - start_time < time_to_move:
        pub.publish(move_command)
        rate.sleep()

    # Create a Twist message to stop the Duckiebot
    stop_command = Twist()
    pub.publish(stop_command)
    rospy.loginfo("Duckiebot stopped.")

if __name__ == '__main__':
    try:
        move_duckiebot()
    except rospy.ROSInterruptException:
        pass

