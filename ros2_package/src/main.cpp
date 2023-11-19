#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "ros2_package/ros2_package_node.hpp"

using namespace ros2_package;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  auto ros2_package_node = std::make_shared<ROS2Package>();

  executor.add_node(ros2_package_node);

  try {
    executor.spin();
  } catch (const std::runtime_error& e) {
    std::cerr << "[ROS2Package] Caught exception: " << e.what() << std::endl;
  }

  std::cout << "[ROS2Package] Shutting down" << std::endl;
  rclcpp::shutdown();
  return 0;
}