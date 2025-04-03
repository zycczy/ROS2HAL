// src/image_proc_hal/rectify_node.hpp
#ifndef IMAGE_PROC_HAL__RECTIFY_NODE_HPP_
#define IMAGE_PROC_HAL__RECTIFY_NODE_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_transport/image_transport.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "image_proc_hal/rectify_processor.hpp"
#include "image_proc_hal/processor_factory.hpp"

namespace image_proc_hal
{

/**
 * @brief ROS 2 node for image rectification using the HAL plugin system
 * 
 * This class demonstrates how to use the HAL plugin system in a ROS 2 node.
 * It subscribes to raw camera images and camera info, processes them
 * using the RectifyProcessor interface, and publishes the rectified images.
 */
class RectifyNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Rectify Node object
   * 
   * @param options Node options
   */
  explicit RectifyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Rectify Node object
   */
  virtual ~RectifyNode();

protected:
  // Parameters
  std::string preferred_vendor_;
  bool use_hardware_acceleration_;
  bool fallback_to_opencv_;

  // Processor instance
  std::shared_ptr<RectifyProcessor> processor_;

  // Subscribers and publisher
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> info_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_pub_;
  
  /**
   * @brief Callback for synchronized image and camera info messages
   * 
   * @param image Raw camera image
   * @param info Camera calibration information
   */
  void image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);
    
  /**
   * @brief Initialize the processor based on node parameters
   * 
   * @return true if initialization succeeded, false otherwise
   */
  bool initialize_processor();
  
  /**
   * @brief Print processor capabilities
   */
  void print_processor_info();
};

}  // namespace image_proc_hal

#endif  // IMAGE_PROC_HAL__RECTIFY_NODE_HPP_