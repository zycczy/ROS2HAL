// src/image_proc_hal/plugins/opencv_debayer_processor.cpp
#include "src/image_proc_hal/plugins/opencv_debayer_processor.hpp"

#include <opencv2/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace image_proc_hal
{
namespace opencv
{

OpenCVDebayerProcessor::OpenCVDebayerProcessor()
{
  initialize_capabilities();
}

void OpenCVDebayerProcessor::initialize_capabilities()
{
  // Set up capabilities specific to OpenCV debayering
  capabilities_.set_hardware_acceleration_supported(false);
  capabilities_.set_vendor_name("OpenCV");
  capabilities_.set_implementation_name("OpenCV Debayering");
  
  // Support Bayer patterns
  capabilities_.add_supported_color_space(ColorSpace::BAYER_RGGB);
  capabilities_.add_supported_color_space(ColorSpace::BAYER_BGGR);
  capabilities_.add_supported_color_space(ColorSpace::BAYER_GBRG);
  capabilities_.add_supported_color_space(ColorSpace::BAYER_GRBG);
  
  // OpenCV has minimal resolution constraints for debayering
  capabilities_.set_min_resolution(2, 2);  // Minimum size for debayering
  capabilities_.set_max_resolution(16384, 16384);  // Typical OpenCV limit
}

int OpenCVDebayerProcessor::convert_algorithm_to_cv_code(DebayerAlgorithm algorithm)
{
  switch (algorithm) {
    case DebayerAlgorithm::NEAREST:
      return cv::COLOR_BayerBG2BGR_NEAREST;  // Base code, pattern is adjusted later
    case DebayerAlgorithm::BILINEAR:
      return cv::COLOR_BayerBG2BGR;  // Base code, pattern is adjusted later
    case DebayerAlgorithm::HQLINEAR:
      // OpenCV doesn't have a specific HQ linear, use Edge-Aware instead
      return cv::COLOR_BayerBG2BGR_EA;
    case DebayerAlgorithm::EDGE_AWARE:
      return cv::COLOR_BayerBG2BGR_EA;
    case DebayerAlgorithm::VNG:
      return cv::COLOR_BayerBG2BGR_VNG;
    default:
      return cv::COLOR_BayerBG2BGR;  // Default to bilinear
  }
}

bool OpenCVDebayerProcessor::debayer(
  const sensor_msgs::msg::Image & bayer_image,
  sensor_msgs::msg::Image & color_image,
  DebayerAlgorithm algorithm)
{
  if (!is_compatible(bayer_image)) {
    return false;
  }

  try {
    // Convert ROS image to OpenCV
    cv_bridge::CvImageConstPtr cv_bayer = cv_bridge::toCvShare(bayer_image);
    
    // Determine the Bayer pattern
    int cv_code = convert_algorithm_to_cv_code(algorithm);
    if (bayer_image.encoding == sensor_msgs::image_encodings::BAYER_RGGB8 ||
        bayer_image.encoding == sensor_msgs::image_encodings::BAYER_RGGB16) {
      cv_code = cv_code - cv::COLOR_BayerBG2BGR + cv::COLOR_BayerRG2BGR;
    } else if (bayer_image.encoding == sensor_msgs::image_encodings::BAYER_BGGR8 ||
               bayer_image.encoding == sensor_msgs::image_encodings::BAYER_BGGR16) {
      cv_code = cv_code - cv::COLOR_BayerBG2BGR + cv::COLOR_BayerBG2BGR;
    } else if (bayer_image.encoding == sensor_msgs::image_encodings::BAYER_GBRG8 ||
               bayer_image.encoding == sensor_msgs::image_encodings::BAYER_GBRG16) {
      cv_code = cv_code - cv::COLOR_BayerBG2BGR + cv::COLOR_BayerGB2BGR;
    } else if (bayer_image.encoding == sensor_msgs::image_encodings::BAYER_GRBG8 ||
               bayer_image.encoding == sensor_msgs::image_encodings::BAYER_GRBG16) {
      cv_code = cv_code - cv::COLOR_BayerBG2BGR + cv::COLOR_BayerGR2BGR;
    } else {
      // Unsupported Bayer pattern
      return false;
    }

    // Create output image
    cv::Mat debayered_image;
    cv::cvtColor(cv_bayer->image, debayered_image, cv_code);

    // Convert back to ROS message
    cv_bridge::CvImage output_cv_image;
    output_cv_image.header = bayer_image.header;
    
    // Determine output encoding based on input bit depth
    if (bayer_image.encoding.find("8") != std::string::npos) {
      output_cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    } else {
      output_cv_image.encoding = sensor_msgs::image_encodings::BGR16;
    }
    
    output_cv_image.image = debayered_image;
    output_cv_image.toImageMsg(color_image);
    
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
  image_proc_hal::opencv::OpenCVDebayerProcessor,
  image_proc_hal::DebayerProcessor)