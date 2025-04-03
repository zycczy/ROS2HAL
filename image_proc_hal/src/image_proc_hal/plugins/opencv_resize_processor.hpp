// src/image_proc_hal/plugins/opencv_resize_processor.hpp
#ifndef IMAGE_PROC_HAL__PLUGINS__OPENCV_RESIZE_PROCESSOR_HPP_
#define IMAGE_PROC_HAL__PLUGINS__OPENCV_RESIZE_PROCESSOR_HPP_

#include "image_proc_hal/resize_processor.hpp"

namespace image_proc_hal
{
namespace opencv
{

/**
 * @brief OpenCV-based implementation of the resize processor
 * 
 * This class provides an implementation of image resizing
 * using OpenCV functions.
 */
class OpenCVResizeProcessor : public ResizeProcessor
{
public:
  /**
   * @brief Construct a new OpenCV Resize Processor object
   */
  OpenCVResizeProcessor();

  /**
   * @brief Destroy the OpenCV Resize Processor object
   */
  ~OpenCVResizeProcessor() override = default;

  /**
   * @brief Resize an image
   * 
   * @param input_image The input image to resize
   * @param output_image The resized output image
   * @param width The desired output width
   * @param height The desired output height
   * @param method The interpolation method to use
   * @return true if successful, false otherwise
   */
  bool resize(
    const sensor_msgs::msg::Image & input_image,
    sensor_msgs::msg::Image & output_image,
    uint32_t width, 
    uint32_t height,
    InterpolationMethod method = InterpolationMethod::LINEAR) override;

private:
  /**
   * @brief Initialize capabilities specific to OpenCV resize
   */
  void initialize_capabilities();

  /**
   * @brief Convert InterpolationMethod to OpenCV interpolation code
   * 
   * @param method The method enum value
   * @return int The corresponding OpenCV interpolation flag
   */
  int convert_method_to_cv_code(InterpolationMethod method);
};

}  // namespace opencv
}  // namespace image_proc_hal

#endif  // IMAGE_PROC_HAL__PLUGINS__OPENCV_RESIZE_PROCESSOR_HPP_