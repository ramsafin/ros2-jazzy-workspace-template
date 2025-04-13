#include <chrono>  // seconds
#include <format>
#include <memory>  // make_shared
#include <string>  // to_string

#include "rclcpp/rclcpp.hpp"  // init, spin, shutdown, Node, TimerBase, Publisher

#include "cpp_publisher/type_adapter.hpp"  // TypeAdapter (specialization)

static constexpr size_t DEFAULT_QOS_HIST_DEPTH = 10;

class AdapterPublisher : public rclcpp::Node
{
public:
  using Adapter = rclcpp::TypeAdapter<std::string, std_msgs::msg::String>;

  explicit AdapterPublisher(size_t pub_rate_secs) : Node("cpp_adapter_publisher")
  {
    publisher_ = this->create_publisher<Adapter>("topic", DEFAULT_QOS_HIST_DEPTH);

    // lambda callback function that publishes new messages to the specified topic
    auto callback = [this]() -> void {
      std::string message = std::format("Data: {}", msg_counter_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.c_str());
      publisher_->publish(message);
    };

    // timer that periocially ivokes the callback function
    timer_ = this->create_wall_timer(std::chrono::seconds(pub_rate_secs), callback);
  }

private:
  size_t msg_counter_{0};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Adapter>::SharedPtr publisher_;
};

auto main(int argc, char* argv[]) -> int
{
  rclcpp::init(argc, argv);

  const size_t publish_rate_secs = 2;
  rclcpp::spin(std::make_shared<AdapterPublisher>(publish_rate_secs));

  rclcpp::shutdown();
  return 0;
}