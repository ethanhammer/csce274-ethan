#!/usr/bin/env python3

import os
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, LanePose

class LaneControllerNode:
    def __init__(self, node_name):
        
        rospy.init_node(node_name, anonymous=True)

        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
    
        self.vel_min = rospy.get_param('project4/vel_min')
        self.vel_max = rospy.get_param('project4/vel_max')
        self.p = rospy.get_param('project4/p')
        self.i = rospy.get_param('project4/i')
        self.d = rospy.get_param('project4/d')

        rospy.logwarn(f"PID Parameters - kp: {self.p}, ki: {self.i}, kd: {self.d}")
        rospy.logwarn(f"Velocity Limits - vel_min: {self.vel_min}, vel_max: {self.vel_max}")

        self.last_time = rospy.Time.now().to_sec()
        self.previous_phi_error = 0.0
        self.previous_d_error = 0.0
        self.integral_phi = 0.0
        self.integral_d = 0.0

        self._vel_right = self.vel_max / 1.5
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self._sub_lane_pose = rospy.Subscriber(f"/{vehicle_name}/lane_filter_node/lane_pose", LanePose, self.cbLanePose, queue_size=1)

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def compute_pid(self, error, dt, previous_error, integral):
      
        integral += error * dt
        derivative = (error - previous_error) / dt if dt > 0 else 0
        output = self.p * error + self.i * integral + self.d * derivative
        return output, integral

    def cbLanePose(self, pose_msg):
        
        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        d_error = pose_msg.d - 0.4  
        phi_error = pose_msg.phi    

        omega_phi, self.integral_phi = self.compute_pid(
            phi_error, dt, self.previous_phi_error, self.integral_phi
        )
        omega_d, self.integral_d = self.compute_pid(
            d_error, dt, self.previous_d_error, self.integral_d
        )

        omega = (omega_phi + omega_d)

	#0.102 is the distance between the wheels
        vel_left = self.clamp(self._vel_right + ( omega * 0.102), self.vel_min, self.vel_max)
        
        duck_name = os.environ.get('VEHICLE_NAME')
        rospy.logwarn(
             f"Duck: {duck_name}, vel_min: {self.vel_min:.2f}, vel_max: {self.vel_max:.2f}, "
             f"vel_left: {vel_left:.2f}, vel_right: {self._vel_right:.2f}, "
             f"p: {self.p:.2f}, i: {self.i:.2f}, d: {self.d:.2f}"
        )

        
        self.previous_phi_error = phi_error
        self.previous_d_error = d_error

        wheels_cmd = WheelsCmdStamped()
        wheels_cmd.vel_left = vel_left
        wheels_cmd.vel_right = self._vel_right
        self._publisher.publish(wheels_cmd)

if __name__ == '__main__':
  
    node = LaneControllerNode(node_name='lane_controller_node')
    rospy.spin()

