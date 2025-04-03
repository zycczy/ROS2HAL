// src/image_proc_hal/plugins/opencv_rectify_processor.hpp
#ifndef IMAGE_PROC_HAL__PLUGINS__OPENCV_RECTIFY_PROCESSOR_HPP_
#define IMAGE_PROC_HAL__PLUGINS__OPENCV_RECTIFY_PROCESSOR_HPP_

#include "image_proc_hal/rectify_processor.hpp"
#include "image_proc_hal/memory_manager.hpp"
#include <memory>

namespace image_proc_hal
{
namespace opencv
{

/**
 * @brief OpenCV-based implementation of the rectify processor
 * 
 * This class provides an implementation of image rectification
 * using OpenCV functions.
 */
class OpenCVRectifyProcessor : public RectifyProcessor
{
public:
  /**
   * @brief Construct a new OpenCV Rectify Processor object
   */
  OpenCVRectifyProcessor();

  /**
   * @brief Destroy the OpenCV Rectify Processor object
   */
  ~OpenCVRectifyProcessor() override = default;

  /**
   * @brief Rectify an image using camera calibration data
   * 
   * @param input_image The distorted input image
   * @param camera_info Camera calibration information
   * @param output_image The rectified output image
   * @return true if successful, false otherwise
   */
  bool rectify(
    const sensor_msgs::msg::Image & input_image,
    const sensor_msgs::msg::CameraInfo & camera_info, 
    sensor_msgs::msg::Image & output_image) override;

private:
  /**
   * @brief Initialize capabilities specific to OpenCV rectification
   */
  void initialize_capabilities();
};

}  // namespace opencv
}  // namespace image_proc_hal

#endif  // IMAGE_PROC_HAL__PLUGINS__OPENCV_RECTIFY_PROCESSOR_HPP_