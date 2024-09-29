#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped

# Set linear velocity in m/s
VELOCITY = 0.3  # Forward linear velocity

class line(DTROS):

    def __init__(self, node_name):
        super(MoveOneMeterNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        # Get vehicle name from environment
        vehicle_name = os.environ['bigbot']
        twist_topic = f"/bigbot/car_cmd_switch_node/cmd"
        
        # Construct publisher
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        
        # Calculate the time to move 1 meter
        self.move_distance = 1.0  # 1 meter
        self.time_to_move = self.move_distance / VELOCITY  # seconds

    def run(self):
        # Create the message
        message = Twist2DStamped(v=VELOCITY, omega=0.0)
        
        # Publish the command
        rospy.sleep(0.5)  # Allow time for the connection to be established
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < self.time_to_move:
            self._publisher.publish(message)
            rospy.sleep(0.1)  # Publish at a reasonable rate
            
        # Stop the robot after moving
        self.stop_robot()

    def stop_robot(self):
        stop_message = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop_message)

if __name__ == '__main__':
    node = MoveOneMeterNode(node_name='move_one_meter_node')
    node.run()
    rospy.spin()

