cmake_minimum_required(VERSION 3.20)
project(cpp_subscriber)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# add executables
add_executable(subscriber_lambda src/lambda.cpp)
ament_target_dependencies(subscriber_lambda rclcpp std_msgs)

add_executable(subscriber_function src/function.cpp)
ament_target_dependencies(subscriber_function rclcpp std_msgs)

add_executable(subscriber_stats src/statistics.cpp)
ament_target_dependencies(subscriber_stats rclcpp std_msgs)

# install targets
install(TARGETS
  subscriber_lambda
  subscriber_function
  subscriber_stats
  DESTINATION lib/${PROJECT_NAME}
)

# setup testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
