#include <chrono>  // seconds
#include <format>
#include <functional>  // bind
#include <memory>      // make_shared

#include "rclcpp/rclcpp.hpp"        // init, spin, shutdown, Node, TimerBase, Publisher
#include "std_msgs/msg/string.hpp"  // String

using namespace std::chrono_literals;

static constexpr size_t DEFAULT_QOS_HIST_DEPTH = 10;
static constexpr std::chrono::seconds DEFAULT_PUBLISH_RATE = 2s;

class FunctionalPublisher : public rclcpp::Node
{
public:
  explicit FunctionalPublisher() : Node("cpp_functional_publisher")
  {
    publisher_ = create_publisher<std_msgs::msg::String>("topic", DEFAULT_QOS_HIST_DEPTH);

    auto callback = std::bind(&FunctionalPublisher::timerCallback, this);
    timer_ = create_wall_timer(DEFAULT_PUBLISH_RATE, callback);

    RCLCPP_INFO(get_logger(), "Publishing messages every %ld secs", DEFAULT_PUBLISH_RATE.count());
  }

private:
  void timerCallback()
  {
    auto message = std_msgs::msg::String();
    message.data = std::format("Data: {}", msg_counter_++);

    publisher_->publish(message);

    RCLCPP_INFO(get_logger(), "Published: '%s'", message.data.c_str());
  }

  size_t msg_counter_{0};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

auto main(int argc, char* argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FunctionalPublisher>());
  rclcpp::shutdown();
  return 0;
}
