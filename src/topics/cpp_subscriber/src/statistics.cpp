#include <chrono>      // seconds
#include <functional>  // placeholder
#include <memory>      // make_shared

#include "rclcpp/rclcpp.hpp"                // init, spin, shutdown, Node
#include "rclcpp/subscription_options.hpp"  // SubscriptionOptions

#include "std_msgs/msg/string.hpp"  // String

using namespace std::chrono_literals;

static constexpr size_t DEFAULT_QOS_HIST_DEPTH = 10;
static constexpr std::chrono::seconds DEFAULT_STATS_PUBLISH_PERIOD = 1s;

class StatisticsSubscriber : public rclcpp::Node
{
public:
  explicit StatisticsSubscriber() : Node("cpp_statistics_subscriber")
  {
    // will publish to /statitics
    auto options = rclcpp::SubscriptionOptions();

    // manually enable topic statistics via options
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

    // configure the collection window and publish period (default 1s)
    options.topic_stats_options.publish_period = DEFAULT_STATS_PUBLISH_PERIOD;

    auto callback = std::bind(&StatisticsSubscriber::callback, this, std::placeholders::_1);

    sub_ = create_subscription<std_msgs::msg::String>(
      "topic", DEFAULT_QOS_HIST_DEPTH, callback, options);

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
  rclcpp::spin(std::make_shared<StatisticsSubscriber>());
  rclcpp::shutdown();
  return 0;
}
