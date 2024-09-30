#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped

def move_duckiebot():
    rospy.init_node('square_mover', anonymous=True)
    pub = rospy.Publisher('/bigbot/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)


    distance = 0.6  
    speed = 0.2
    time_to_move = distance / speed  
    turn_speed = 4.5
    turn_time = 1.4 / turn_speed  

    move_command = Twist2DStamped()
    move_command.header.stamp = rospy.Time.now()
    move_command.v = speed
    move_command.omega = 0

    turn_command = Twist2DStamped()
    turn_command.header.stamp = rospy.Time.now()
    turn_command.v = 0
    turn_command.omega = turn_speed

    stop_command = Twist2DStamped()
    stop_command.header.stamp = rospy.Time.now()
    stop_command.v = 0
    stop_command.omega = 0

    rate = rospy.Rate(100) 

    for _ in range(4):  
    
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < time_to_move:
            move_command.header.stamp = rospy.Time.now()
            pub.publish(move_command)
            rate.sleep()

      
        pub.publish(stop_command)
        rospy.sleep(0.5)  

        # Turn 90 degrees
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < turn_time:
            turn_command.header.stamp = rospy.Time.now()
            pub.publish(turn_command)
            rate.sleep()

        pub.publish(stop_command)
        rospy.sleep(0.5)  


if __name__ == '__main__':
    try:
        move_duckiebot()
    except rospy.ROSInterruptException:
        pass

