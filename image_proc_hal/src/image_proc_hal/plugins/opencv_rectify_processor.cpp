// src/image_proc_hal/plugins/opencv_rectify_processor.cpp
#include "src/image_proc_hal/plugins/opencv_rectify_processor.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
#include "pluginlib/class_list_macros.hpp"

namespace image_proc_hal
{
namespace opencv
{

OpenCVRectifyProcessor::OpenCVRectifyProcessor()
{
  initialize_capabilities();
}

void OpenCVRectifyProcessor::initialize_capabilities()
{
  // Set up capabilities specific to OpenCV rectification
  capabilities_.set_hardware_acceleration_supported(false);
  capabilities_.set_vendor_name("OpenCV");
  capabilities_.set_implementation_name("OpenCV Rectification");
  
  // Support common image formats
  capabilities_.add_supported_color_space(ColorSpace::RGB);
  capabilities_.add_supported_color_space(ColorSpace::BGR);
  capabilities_.add_supported_color_space(ColorSpace::GRAY);
  
  // OpenCV has minimal resolution constraints
  capabilities_.set_min_resolution(1, 1);
  capabilities_.set_max_resolution(16384, 16384);  // Typical OpenCV limit
}

bool OpenCVRectifyProcessor::rectify(
  const sensor_msgs::msg::Image & input_image,
  const sensor_msgs::msg::CameraInfo & camera_info,
  sensor_msgs::msg::Image & output_image)
{
  if (!is_compatible(input_image)) {
    return false;
  }

  try {
    // Convert ROS image to OpenCV
    cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(input_image);
    
    // Set up camera model
    image_geometry::PinholeCameraModel camera_model;
    camera_model.fromCameraInfo(camera_info);

    // Create output image
    cv::Mat rectified_image;
    
    // Apply rectification
    if (cv_image->encoding == sensor_msgs::image_encodings::MONO8 ||
        cv_image->encoding == sensor_msgs::image_encodings::MONO16) {
      // For mono images
      cv::remap(
        cv_image->image, 
        rectified_image,
        camera_model.rectifyMapX(),
        camera_model.rectifyMapY(),
        cv::INTER_LINEAR);
    } else {
      // For color images
      cv::remap(
        cv_image->image, 
        rectified_image,
        camera_model.rectifyMapX(),
        camera_model.rectifyMapY(),
        cv::INTER_LINEAR);
    }

    // Convert back to ROS message
    cv_bridge::CvImage output_cv_image;
    output_cv_image.header = input_image.header;
    output_cv_image.encoding = input_image.encoding;
    output_cv_image.image = rectified_image;
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
  image_proc_hal::opencv::OpenCVRectifyProcessor,
  image_proc_hal::RectifyProcessor)