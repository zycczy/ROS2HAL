// src/rectify_node_main.cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "src/image_proc_hal/rectify_node.hpp"

int main(int argc, char * argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Create the rectify node
  rclcpp::NodeOptions options;
  auto node = std::make_shared<image_proc_hal::RectifyNode>(options);
  
  // Spin the node
  rclcpp::spin(node);
  
  // Shutdown
  rclcpp::shutdown();
  return 0;
}