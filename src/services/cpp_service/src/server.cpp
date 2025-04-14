#include <memory>  // make_shared

#include "rclcpp/rclcpp.hpp"         // init, spin, shutdown, Node
#include "std_srvs/srv/trigger.hpp"  // Trigger

class ServiceServer : public rclcpp::Node
{
public:
  ServiceServer() : Node("service_server")
  {
    RCLCPP_INFO(get_logger(), "Starting service server...");

    auto callback = [this](
                      const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                      std_srvs::srv::Trigger::Response::SharedPtr resp) -> void {
      const bool success = executeOperation();

      resp->success = success;
      resp->message = success ? "operation succeeded" : "operation failed";

      RCLCPP_INFO(get_logger(), "Service executed. Result: %s", resp->message.c_str());
    };

    service_ = create_service<std_srvs::srv::Trigger>("execute_service", callback);

    RCLCPP_INFO(get_logger(), "Service server ready. Waiting for requests...");
  }

private:
  auto executeOperation() -> bool
  {
    RCLCPP_INFO(get_logger(), "Executing operation: %ld...", operation_count_);

    // Simulate some operation with alternating success/failure
    const bool status = operation_count_ % 2 == 0;
    operation_count_ += 1;

    if (!status) {
      RCLCPP_WARN(get_logger(), "Operation simulated failure");
    }

    return status;
  }

  size_t operation_count_{0};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

auto main(int argc, char** argv) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceServer>());
  rclcpp::shutdown();

  return 0;
}