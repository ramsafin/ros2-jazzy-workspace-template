#include <functional>  // placeholder
#include <memory>      // make_shared

#include "rclcpp/rclcpp.hpp"        // init, spin, shutdown, Node
#include "std_msgs/msg/string.hpp"  // String

static constexpr size_t DEFAULT_QOS_HIST_DEPTH = 10;
class FunctionalSubscriber : public rclcpp::Node
{
public:
  explicit FunctionalSubscriber() : Node("cpp_functional_subscriber")
  {
    auto callback = std::bind(&FunctionalSubscriber::callback, this, std::placeholders::_1);
    sub_ = create_subscription<std_msgs::msg::String>("topic", DEFAULT_QOS_HIST_DEPTH, callback);
    RCLCPP_INFO(get_logger(), "Created subscriber");
  }

private:
  void callback(std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(get_logger(), "Got message: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

auto main(int argc, char* argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FunctionalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
