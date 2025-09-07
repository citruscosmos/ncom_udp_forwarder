cmake_minimum_required(VERSION 3.8)
project(ncom_udp_forwarder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(oxts_msgs REQUIRED)

# add executable
add_executable(ncom_forwarder_node src/ncom_forwarder_node.cpp)
ament_target_dependencies(
  ncom_forwarder_node
  "rclcpp"
  "oxts_msgs"
)

# install executable
install(
  TARGETS ncom_forwarder_node
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

