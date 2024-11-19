#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose

class SimpleLaneControllerNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)

        self.pub_car_cmd = rospy.Publisher("/bigbot/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        self.sub_lane_pose = rospy.Subscriber("/bigbot/lane_filter_node/lane_pose", LanePose, self.cbLanePose, queue_size=1)

        self.kp_position = 18
        self.ki_position = 0.02
        self.kd_position = 0.75
       
        self.kp_heading = 12
        self.ki_heading = 0.02
        self.kd_heading = 0.5  

        self.integral_position = 0
        self.integral_heading = 0
        self.previous_error_position = 0
        self.previous_error_heading = 0

        self.last_time = rospy.Time.now().to_sec()

        self.max_error = 16  
        self.max_integral = 0.5  
        self.max_omega = 1000  

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def compute_pid(self, error, dt, previous_error, integral, kp, ki, kd):
    
        error = self.clamp(error, -self.max_error, self.max_error)

        integral += error * dt
        integral = self.clamp(integral, -self.max_integral, self.max_integral)

        derivative = (error - previous_error) / dt if dt > 0 else 0

        output = kp * error + ki * integral + kd * derivative

        return output, integral

    def cbLanePose(self, pose_msg):
    
        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.last_time
        self.last_time = current_time

        d_err = pose_msg.d - 0.4  
        phi_err = pose_msg.phi

        rospy.logwarn("WILLIAM HAMMER LANE FOLLOWING CODE: d=%.2f, phi=%.2f" % (d_err, phi_err))

        omega, self.integral_heading = self.compute_pid(
            phi_err, dt, 
            self.previous_error_heading, 
            self.integral_heading, 
            self.kp_heading, 
            self.ki_heading, 
            self.kd_heading
        )

        omega = self.clamp(omega, -self.max_omega, self.max_omega)

        self.previous_error_heading = phi_err
        self.previous_error_position = d_err

        v = 0.35

        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega * -1 
        self.pub_car_cmd.publish(car_cmd_msg)

if __name__ == "__main__":
    lane_controller_node = SimpleLaneControllerNode(node_name="lane_controller")
    rospy.spin()

