#include <chrono>  // seconds
#include <format>
#include <memory>  // make_shared
#include <string>

#include "rclcpp/rclcpp.hpp"  // init, spin, shutdown, Node, TimerBase, Publisher

#include "cpp_publisher/type_adapter.hpp"  // StringAdapter

using namespace std::chrono_literals;

static constexpr size_t DEFAULT_QOS_HIST_DEPTH = 10;
static constexpr std::chrono::seconds DEFAULT_PUBLISH_RATE = 2s;

class AdapterPublisher : public rclcpp::Node
{
public:
  explicit AdapterPublisher() : Node("cpp_adapter_publisher")
  {
    publisher_ = create_publisher<StringAdapter>("topic", DEFAULT_QOS_HIST_DEPTH);

    // lambda callback function that publishes new messages to the specified topic
    auto callback = [this]() {
      const std::string message = std::format("Data: {}", msg_counter_++);
      publisher_->publish(message);

      RCLCPP_INFO(get_logger(), "Published: '%s'", message.c_str());
    };

    // timer that periocially ivokes the callback function
    timer_ = create_wall_timer(DEFAULT_PUBLISH_RATE, callback);
    RCLCPP_INFO(get_logger(), "Publishing messages every %ld secs", DEFAULT_PUBLISH_RATE.count());
  }

private:
  size_t msg_counter_{0};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<StringAdapter>::SharedPtr publisher_;
};

auto main(int argc, char* argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdapterPublisher>());
  rclcpp::shutdown();
  return 0;
}