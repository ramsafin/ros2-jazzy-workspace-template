#pragma once

#include <format>
#include <string>

#include "rclcpp/type_adapter.hpp"  // TypeAdapter
#include "std_msgs/msg/string.hpp"  // String

template <>
struct rclcpp::TypeAdapter<std::string, std_msgs::msg::String>
{
  using is_specialized = std::true_type;

  using custom_type = std::string;
  using ros_message_type = std_msgs::msg::String;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.data = std::format("[std::string]: {}", src);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst = std::format("[std_msgs::String]: {}", src.data);
  }
};

using StringAdapter = rclcpp::TypeAdapter<std::string, std_msgs::msg::String>;