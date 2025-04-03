// src/image_proc_hal/plugins/opencv_resize_processor.cpp
#include "src/image_proc_hal/plugins/opencv_resize_processor.hpp"

#include <opencv2/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"
#include "pluginlib/class_list_macros.hpp"

namespace image_proc_hal
{
namespace opencv
{

OpenCVResizeProcessor::OpenCVResizeProcessor()
{
  initialize_capabilities();
}

void OpenCVResizeProcessor::initialize_capabilities()
{
  // Set up capabilities specific to OpenCV resize
  capabilities_.set_hardware_acceleration_supported(false);
  capabilities_.set_vendor_name("OpenCV");
  capabilities_.set_implementation_name("OpenCV Resize");
  
  // Support common image formats
  capabilities_.add_supported_color_space(ColorSpace::RGB);
  capabilities_.add_supported_color_space(ColorSpace::BGR);
  capabilities_.add_supported_color_space(ColorSpace::GRAY);
  capabilities_.add_supported_color_space(ColorSpace::BAYER_RGGB);
  capabilities_.add_supported_color_space(ColorSpace::BAYER_BGGR);
  capabilities_.add_supported_color_space(ColorSpace::BAYER_GBRG);
  capabilities_.add_supported_color_space(ColorSpace::BAYER_GRBG);
  
  // OpenCV has minimal resolution constraints
  capabilities_.set_min_resolution(1, 1);
  capabilities_.set_max_resolution(16384, 16384);  // Typical OpenCV limit
}

int OpenCVResizeProcessor::convert_method_to_cv_code(InterpolationMethod method)
{
  switch (method) {
    case InterpolationMethod::NEAREST:
      return cv::INTER_NEAREST;
    case InterpolationMethod::LINEAR:
      return cv::INTER_LINEAR;
    case InterpolationMethod::CUBIC:
      return cv::INTER_CUBIC;
    case InterpolationMethod::LANCZOS:
      return cv::INTER_LANCZOS4;
    default:
      return cv::INTER_LINEAR;  // Default to linear interpolation
  }
}

bool OpenCVResizeProcessor::resize(
  const sensor_msgs::msg::Image & input_image,
  sensor_msgs::msg::Image & output_image,
  uint32_t width, 
  uint32_t height,
  InterpolationMethod method)
{
  if (!is_compatible(input_image)) {
    return false;
  }

  try {
    // Convert ROS image to OpenCV
    cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(input_image);
    
    // Create output image
    cv::Mat resized_image;
    
    // Perform resize operation
    cv::resize(
      cv_image->image, 
      resized_image, 
      cv::Size(width, height), 
      0, 0, 
      convert_method_to_cv_code(method));
    
    // Convert back to ROS message
    cv_bridge::CvImage output_cv_image;
    output_cv_image.header = input_image.header;
    output_cv_image.encoding = input_image.encoding;
    output_cv_image.image = resized_image;
    output_cv_image.toImageMsg(output_image);
    
    return true;
  } catch (const cv_bridge::Exception & e) {
    // Handle conversion error
    return false;
  } catch (const std::exception & e) {
    // Handle other errors
    return false;
  }
}

}  // namespace opencv
}  // namespace image_proc_hal

// Register this implementation as a plugin
PLUGINLIB_EXPORT_CLASS(
  image_proc_hal::opencv::OpenCVResizeProcessor,
  image_proc_hal::ResizeProcessor)