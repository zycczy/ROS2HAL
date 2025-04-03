// src/image_proc_hal/plugins/opencv_debayer_processor.hpp
#ifndef IMAGE_PROC_HAL__PLUGINS__OPENCV_DEBAYER_PROCESSOR_HPP_
#define IMAGE_PROC_HAL__PLUGINS__OPENCV_DEBAYER_PROCESSOR_HPP_

#include "image_proc_hal/debayer_processor.hpp"

namespace image_proc_hal
{
namespace opencv
{

/**
 * @brief OpenCV-based implementation of the debayer processor
 * 
 * This class provides an implementation of Bayer pattern demosaicing
 * using OpenCV functions.
 */
class OpenCVDebayerProcessor : public DebayerProcessor
{
public:
  /**
   * @brief Construct a new OpenCV Debayer Processor object
   */
  OpenCVDebayerProcessor();

  /**
   * @brief Destroy the OpenCV Debayer Processor object
   */
  ~OpenCVDebayerProcessor() override = default;

  /**
   * @brief Convert a Bayer pattern image to a color image
   * 
   * @param bayer_image The Bayer pattern input image
   * @param color_image The color output image
   * @param algorithm The debayering algorithm to use
   * @return true if successful, false otherwise
   */
  bool debayer(
    const sensor_msgs::msg::Image & bayer_image,
    sensor_msgs::msg::Image & color_image,
    DebayerAlgorithm algorithm = DebayerAlgorithm::BILINEAR) override;

private:
  /**
   * @brief Initialize capabilities specific to OpenCV debayering
   */
  void initialize_capabilities();

  /**
   * @brief Convert DebayerAlgorithm to OpenCV debayering code
   * 
   * @param algorithm The algorithm enum value
   * @return int The corresponding OpenCV flag
   */
  int convert_algorithm_to_cv_code(DebayerAlgorithm algorithm);
};

}  // namespace opencv
}  // namespace image_proc_hal

#endif  // IMAGE_PROC_HAL__PLUGINS__OPENCV_DEBAYER_PROCESSOR_HPP_