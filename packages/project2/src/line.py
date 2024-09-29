#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped  # Adjust package name as necessary
import time

def move_duckiebot():
    rospy.init_node('line', anonymous=True)
    pub = rospy.Publisher('/bigbot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)

    # Parameters
    distance = 1.0  # 1 meter
    speed = 0.3     # m/s
    time_to_move = distance / speed  # time in seconds

    # Create a Twist2DStamped message for forward movement
    move_command = Twist2DStamped()
    move_command.header.stamp = rospy.Time.now()
    move_command.v = speed  # Set linear velocity
    move_command.omega = 0  # Set angular velocity

    # Publish the move command
    rospy.loginfo("Moving Duckiebot forward...")
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.get_time()

    while rospy.get_time() - start_time < time_to_move:
        move_command.header.stamp = rospy.Time.now()  # Update the timestamp
        pub.publish(move_command)
        rate.sleep()

    # Create a stop command
    stop_command = Twist2DStamped()
    stop_command.header.stamp = rospy.Time.now()
    stop_command.v = 0
    stop_command.omega = 0
    pub.publish(stop_command)
    rospy.loginfo("Duckiebot stopped.")

if __name__ == '__main__':
    try:
        move_duckiebot()
    except rospy.ROSInterruptException:
        pass

