#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class IntPublisherNode : public rclcpp::Node
{
public:
  IntPublisherNode() : Node("int_publisher_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("int_topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        auto message = std_msgs::msg::Int32();
        message.data = 42;  // Example integer value
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IntPublisherNode>());
  rclcpp::shutdown();
  return 0;
}

