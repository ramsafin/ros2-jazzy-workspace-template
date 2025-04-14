#include <memory>  // make_shared

#include "rclcpp/rclcpp.hpp"        // init, spin, shutdown, Node
#include "std_msgs/msg/string.hpp"  // String

static constexpr size_t DEFAULT_QOS_HIST_DEPTH = 10;

class LambdaSubscriber : public rclcpp::Node
{
public:
  explicit LambdaSubscriber() : Node("cpp_lambda_subscriber")
  {
    auto callback = [this](std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO(get_logger(), "Got message: '%s'", msg->data.c_str());
    };

    sub_ = create_subscription<std_msgs::msg::String>("topic", DEFAULT_QOS_HIST_DEPTH, callback);
    RCLCPP_INFO(get_logger(), "Created subscriber");
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

auto main(int argc, char* argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LambdaSubscriber>());
  rclcpp::shutdown();
  return 0;
}