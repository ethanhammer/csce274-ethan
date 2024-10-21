#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose

class SimpleLaneControllerNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)

        self.pub_car_cmd = rospy.Publisher("/bigbot/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        self.sub_lane_pose = rospy.Subscriber("/bigbot/lane_filter_node/lane_pose", LanePose, self.cbLanePose, queue_size=1)

        self.kp_position = 3.5
        self.ki_position = 0.0
        self.kd_position = 0.0
        self.kp_heading = 3.5
        self.ki_heading = 0.0
        self.kd_heading = 0.0

        self.integral_position = 0
        self.integral_heading = 0
        self.previous_error_position = 0
        self.previous_error_heading = 0

        self.last_time = rospy.Time.now().to_sec()

    def compute_pid(self, error, dt, previous_error, integral, kp, ki, kd):
        integral += error * dt
        derivative = (error - previous_error) / dt if dt > 0 else 0
        output = kp * error + ki * integral + kd * derivative
        return output, integral

    def cbLanePose(self, pose_msg):
        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Compute errors
        d_err = pose_msg.d  # Lateral deviation
        phi_err = pose_msg.phi  # Heading error

        # Compute control outputs using PID controllers
        v = 0.5  # Constant forward velocity
        omega, self.integral_heading = self.compute_pid(phi_err, dt, self.previous_error_heading, self.integral_heading, self.kp_heading, self.ki_heading, self.kd_heading)
        speed_adjustment, self.integral_position = self.compute_pid(d_err, dt, self.previous_error_position, self.integral_position, self.kp_position, self.ki_position, self.kd_position)
        v += speed_adjustment

        # Update previous errors
        self.previous_error_heading = phi_err
        self.previous_error_position = d_err
        
        # Publish command
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega * -1
        self.pub_car_cmd.publish(car_cmd_msg)

if __name__ == "__main__":
    lane_controller_node = SimpleLaneControllerNode(node_name="lane_controller")
    rospy.spin()

