#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose, StopLineReading

class SimplePID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

class SimpleLaneControllerNode:
    def __init__(self, node_name):
        # Initialize the ROS node
        rospy.init_node(node_name)

        # Publisher for car commands
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscribers for lane pose and stop line readings
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.cbLanePose, queue_size=1)
        self.sub_stop_line = rospy.Subscriber("~stop_line_reading", StopLineReading, self.cbStopLineReading, queue_size=1)

        # Initialize PID controller parameters
        self.pid_position = SimplePID(kp=1.0, ki=0.0, kd=0.1)  # Lateral position PID
        self.pid_heading = SimplePID(kp=2.0, ki=0.0, kd=0.5)   # Heading PID

        self.at_stop_line = False
        self.last_time = rospy.Time.now().to_sec()

        rospy.loginfo("Simple Lane Controller Node Initialized!")

    def cbLanePose(self, pose_msg):
        """Callback for lane pose messages."""
        if not self.at_stop_line:
            current_time = rospy.Time.now().to_sec()
            dt = current_time - self.last_time
            self.last_time = current_time
            
            # Compute errors
            d_err = pose_msg.d  # Lateral deviation
            phi_err = pose_msg.phi  # Heading error

            # Compute control outputs using PID controllers
            v = 1.0  # Constant forward velocity
            omega = self.pid_heading.compute(phi_err, dt)  # Control for heading
            # Optionally adjust speed based on lateral deviation
            speed_adjustment = self.pid_position.compute(d_err, dt)
            v += speed_adjustment

            # Publish command
            car_cmd_msg = Twist2DStamped()
            car_cmd_msg.v = v
            car_cmd_msg.omega = omega
            self.publishCmd(car_cmd_msg)

    def cbStopLineReading(self, msg):
        """Callback for stop line readings."""
        self.at_stop_line = msg.stop_line_detected
        if self.at_stop_line:
            # Stop the car if at stop line
            car_cmd_msg = Twist2DStamped()
            car_cmd_msg.v = 0.0  # Stop
            car_cmd_msg.omega = 0.0  # No rotation
            self.publishCmd(car_cmd_msg)

    def publishCmd(self, car_cmd_msg):
        self.pub_car_cmd.publish(car_cmd_msg)

if __name__ == "__main__":
    # Create and run the simple lane controller node
    lane_controller_node = SimpleLaneControllerNode(node_name="simple_lane_controller_node")
    rospy.spin()

