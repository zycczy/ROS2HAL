// src/image_proc_hal/rectify_node.cpp
#include "src/image_proc_hal/rectify_node.hpp"

namespace image_proc_hal
{

RectifyNode::RectifyNode(const rclcpp::NodeOptions & options)
: Node("rectify_node", options)
{
  // Declare and get parameters
  this->declare_parameter("use_hardware_acceleration", true);
  this->declare_parameter("preferred_vendor", "");
  this->declare_parameter("fallback_to_opencv", true);
  
  use_hardware_acceleration_ = this->get_parameter("use_hardware_acceleration").as_bool();
  preferred_vendor_ = this->get_parameter("preferred_vendor").as_string();
  fallback_to_opencv_ = this->get_parameter("fallback_to_opencv").as_bool();

  // Initialize processor
  if (!initialize_processor()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize image processor");
    throw std::runtime_error("Failed to initialize image processor");
  }
  
  // Print processor information
  print_processor_info();
  
  // Set up publishers and subscribers
  rect_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_rect", 10);
  
  // Use message filters to synchronize image and camera_info
  image_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "image_raw");
  info_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
    this, "camera_info");
  
  // Create synchronizer
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), *image_sub_, *info_sub_);
  
  // Register callback
  sync_->registerCallback(
    std::bind(&RectifyNode::image_callback, this, std::placeholders::_1, std::placeholders::_2));
    
  RCLCPP_INFO(this->get_logger(), "Rectify node initialized");
  RCLCPP_INFO(this->get_logger(), "Subscribed to: %s and %s", 
              image_sub_->getTopic().c_str(), info_sub_->getTopic().c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing to: %s", rect_pub_->get_topic_name());
}

RectifyNode::~RectifyNode()
{
  // Release resources
}

bool RectifyNode::initialize_processor()
{
  // Get available implementations
  auto & factory = ProcessorFactory::get_instance();
  auto available_impls = factory.get_available_implementations<RectifyProcessor>();
  
  if (available_impls.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No rectify processor implementations found");
    return false;
  }
  
  RCLCPP_INFO(this->get_logger(), "Found %zu rectify processor implementations", 
              available_impls.size());
  for (const auto & impl : available_impls) {
    RCLCPP_INFO(this->get_logger(), " - %s", impl.c_str());
  }
  
  // Try to create processor with preferred vendor if specified
  if (!preferred_vendor_.empty()) {
    processor_ = factory.create_processor<RectifyProcessor>(preferred_vendor_);
    if (processor_) {
      RCLCPP_INFO(this->get_logger(), "Using preferred vendor: %s", preferred_vendor_.c_str());
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to create processor from preferred vendor: %s", 
                 preferred_vendor_.c_str());
    }
  }
  
  // If preferred vendor not available or not specified, choose based on hardware acceleration preference
  if (use_hardware_acceleration_) {
    // Try to find hardware-accelerated implementation
    for (const auto & impl : available_impls) {
      auto proc = factory.create_processor<RectifyProcessor>(impl);
      if (proc && proc->get_capabilities().supports_hardware_acceleration()) {
        processor_ = proc;
        RCLCPP_INFO(this->get_logger(), "Using hardware-accelerated implementation: %s", 
                   impl.c_str());
        return true;
      }
    }
    RCLCPP_WARN(this->get_logger(), "No hardware-accelerated implementation found");
  }
  
  // Fall back to OpenCV if allowed
  if (fallback_to_opencv_) {
    for (const auto & impl : available_impls) {
      if (impl.find("OpenCV") != std::string::npos) {
        processor_ = factory.create_processor<RectifyProcessor>(impl);
        if (processor_) {
          RCLCPP_INFO(this->get_logger(), "Falling back to OpenCV implementation: %s", 
                     impl.c_str());
          return true;
        }
      }
    }
  }
  
  // If all else fails, use the first available implementation
  processor_ = factory.create_processor<RectifyProcessor>(available_impls[0]);
  if (processor_) {
    RCLCPP_INFO(this->get_logger(), "Using default implementation: %s", 
               available_impls[0].c_str());
    return true;
  }
  
  return false;
}

void RectifyNode::print_processor_info()
{
  if (!processor_) {
    return;
  }
  
  auto caps = processor_->get_capabilities();
  RCLCPP_INFO(this->get_logger(), "Using processor: %s from vendor %s", 
             caps.get_implementation_name().c_str(), caps.get_vendor_name().c_str());
  RCLCPP_INFO(this->get_logger(), "Hardware acceleration: %s", 
             caps.supports_hardware_acceleration() ? "YES" : "NO");
  
  auto min_res = caps.get_min_resolution();
  auto max_res = caps.get_max_resolution();
  RCLCPP_INFO(this->get_logger(), "Resolution range: %dx%d to %dx%d", 
             min_res.first, min_res.second, max_res.first, max_res.second);
  
  auto color_spaces = caps.get_supported_color_spaces();
  std::string color_space_str = "Supported color spaces: ";
  for (const auto & cs : color_spaces) {
    color_space_str += to_string(cs) + " ";
  }
  RCLCPP_INFO(this->get_logger(), "%s", color_space_str.c_str());
}

void RectifyNode::image_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr& image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
{
  if (!processor_) {
    RCLCPP_ERROR(this->get_logger(), "Processor not initialized");
    return;
  }
  
  sensor_msgs::msg::Image rect_image;
  auto start_time = this->now();
  
  // Process the image using the selected processor
  bool success = processor_->rectify(*image, *info, rect_image);
  
  auto end_time = this->now();
  auto processing_time = (end_time - start_time).seconds() * 1000.0;
  
  if (success) {
    RCLCPP_DEBUG(this->get_logger(), "Image rectified in %.2f ms", processing_time);
    rect_pub_->publish(rect_image);
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to rectify image");
  }
}

}  // namespace image_proc_hal