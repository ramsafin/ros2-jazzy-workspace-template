cmake_minimum_required(VERSION 3.20)
project(cpp_service_server)

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
find_package(std_srvs REQUIRED)

# add executables
add_executable(service_server src/server.cpp)
ament_target_dependencies(service_server rclcpp std_srvs)

# install targets
install(TARGETS
  service_server
  DESTINATION lib/${PROJECT_NAME}
)

# setup testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
