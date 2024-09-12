import rospy
from std_msgs.msg import Int32

def callback(msg):
    # This function will be called whenever a new message is received
    rospy.loginfo("Received: %d", msg.data)

def listener():
    # Initialize the ROS node
    rospy.init_node('subscriber_node', anonymous=True)
    
    # Create a subscriber to the "total" topic with std_msgs/Int32 messages
    rospy.Subscriber('total', Int32, callback)
    
    # Keep the node running to process incoming messages
    rospy.spin()

if __name__ == '__main__':
    listener()
