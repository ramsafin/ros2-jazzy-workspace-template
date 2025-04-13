cmake_minimum_required(VERSION 3.20)
project(cpp_publisher)

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
add_executable(publisher_lambda src/lambda.cpp)
ament_target_dependencies(publisher_lambda rclcpp std_msgs)

add_executable(publisher_function src/function.cpp)
ament_target_dependencies(publisher_function rclcpp std_msgs)

add_executable(publisher_adapter src/adapter.cpp)
ament_target_dependencies(publisher_adapter rclcpp std_msgs)

target_include_directories(publisher_adapter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# export include for downstream packages
ament_export_include_directories(include)

# install targets
install(TARGETS
  publisher_lambda
  publisher_function
  publisher_adapter
  DESTINATION lib/${PROJECT_NAME}
)

# setup testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
