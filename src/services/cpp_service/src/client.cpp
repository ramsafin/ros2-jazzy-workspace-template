#include <chrono>  // seconds
#include <memory>  // make_shared

#include "rclcpp/rclcpp.hpp"         // init, spin, shutdown, Node,
#include "std_srvs/srv/trigger.hpp"  // Trigger

using namespace std::chrono_literals;

class ServiceClient : public rclcpp::Node
{
public:
  ServiceClient() : Node("service_client")
  {
    client_ = create_client<std_srvs::srv::Trigger>("execute_service");
    RCLCPP_INFO(get_logger(), "Initialized service client");
  }

  auto waitForService(std::chrono::seconds timeout = 1s) const -> bool
  {
    while (!client_->wait_for_service(timeout)) {
      if (!rclcpp::ok()) {
        RCLCPP_WARN(get_logger(), "Interrupted while waiting for service");
        return false;
      }
      RCLCPP_INFO(get_logger(), "Service not available, waiting...");
    }
    return true;
  }

  auto callService() -> std::optional<std_srvs::srv::Trigger::Response>
  {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client_->async_send_request(req);

    const auto status = rclcpp::spin_until_future_complete(get_node_base_interface(), future);

    switch (status) {
      case rclcpp::FutureReturnCode::SUCCESS: {
        auto resp = future.get();
        RCLCPP_INFO(get_logger(), "Service response: %s", resp->message.c_str());
        return *resp;
      }
      case rclcpp::FutureReturnCode::INTERRUPTED:
        RCLCPP_ERROR(get_logger(), "Service call interrupted");
        break;
      case rclcpp::FutureReturnCode::TIMEOUT:
        RCLCPP_ERROR(get_logger(), "Service call timed out");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Service call failed");
    }
    return std::nullopt;
  }

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

auto main(int argc, char** argv) -> int
{
  rclcpp::init(argc, argv);

  auto client = std::make_shared<ServiceClient>();

  if (!client->waitForService(5s)) {
    RCLCPP_ERROR(client->get_logger(), "Failed to connect to service");
    rclcpp::shutdown();
    return 1;
  }

  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

  if (auto resp = client->callService()) {
    RCLCPP_INFO(client->get_logger(), "Service succeeded: %s", resp->success ? "true" : "false");
  } else {
    RCLCPP_ERROR(client->get_logger(), "Service call failed");
  }

  rclcpp::shutdown();
  return 0;
}