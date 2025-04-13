#include <chrono>  // seconds
#include <format>
#include <functional>  // bind
#include <memory>      // make_shared
#include <string>      // to_string

#include "rclcpp/rclcpp.hpp"        // init, spin, shutdown, Node, TimerBase, Publisher
#include "std_msgs/msg/string.hpp"  // String

static constexpr size_t DEFAULT_QOS_HIST_DEPTH = 10;

class FunctionalPublisher : public rclcpp::Node
{
public:
  explicit FunctionalPublisher(size_t pub_rate_secs) : Node("cpp_functional_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", DEFAULT_QOS_HIST_DEPTH);

    const auto period = std::chrono::seconds(pub_rate_secs);
    timer_ = this->create_wall_timer(period, std::bind(&FunctionalPublisher::timerCallback, this));
  }

private:
  void timerCallback()
  {
    auto message = std_msgs::msg::String();
    message.data = std::format("Data: {}", msg_counter_++);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  size_t msg_counter_{0};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

auto main(int argc, char* argv[]) -> int
{
  rclcpp::init(argc, argv);

  const size_t publish_rate_secs = 2;
  rclcpp::spin(std::make_shared<FunctionalPublisher>(publish_rate_secs));

  rclcpp::shutdown();
  return 0;
}
