#include <format>
#include <memory>  // make_shared, shared_ptr

#include "rclcpp/rclcpp.hpp"         // init, spin, shutdown, Node
#include "std_srvs/srv/trigger.hpp"  // Trigger

class ServiceServer : public rclcpp::Node
{
  using Trigger = std_srvs::srv::Trigger;
  using Request = std::shared_ptr<Trigger::Request>;
  using Response = std::shared_ptr<Trigger::Response>;

public:
  ServiceServer() : Node("service_server")
  {
    auto callback = [this](Request, Response resp) -> void {
      const bool success = execute();

      resp->success = success;
      resp->message = std::format("Status: {}", success);

      RCLCPP_INFO(this->get_logger(), "Sending back response: status = %s", resp->message.c_str());
    };

    service_ = this->create_service<std_srvs::srv::Trigger>("execute_service", callback);

    RCLCPP_INFO(this->get_logger(), "Waiting for clients...");
  }

private:
  bool execute()
  {
    RCLCPP_INFO(this->get_logger(), "Executing operations...");

    const bool status = index_ % 2 == 0;
    index_ += 1;

    return status;
  }

  size_t index_{0};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

auto main(int argc, char** argv) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceServer>());
  rclcpp::shutdown();
  return 0;
}