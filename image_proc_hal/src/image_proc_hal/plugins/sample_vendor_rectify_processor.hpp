// src/image_proc_hal/plugins/sample_vendor_rectify_processor.hpp
#ifndef IMAGE_PROC_HAL__PLUGINS__SAMPLE_VENDOR_RECTIFY_PROCESSOR_HPP_
#define IMAGE_PROC_HAL__PLUGINS__SAMPLE_VENDOR_RECTIFY_PROCESSOR_HPP_

#include "image_proc_hal/rectify_processor.hpp"
#include "src/image_proc_hal/plugins/opencv_rectify_processor.hpp"
#include <memory>

namespace image_proc_hal
{
namespace sample_vendor
{

/**
 * @brief Sample vendor implementation of the rectify processor
 * 
 * This class demonstrates how a hardware vendor could implement
 * the RectifyProcessor interface to provide hardware acceleration.
 * It includes handling for hardware-specific limitations and fallback
 * to OpenCV when necessary.
 */
class SampleVendorRectifyProcessor : public RectifyProcessor
{
public:
  /**
   * @brief Construct a new Sample Vendor Rectify Processor object
   */
  SampleVendorRectifyProcessor();

  /**
   * @brief Destroy the Sample Vendor Rectify Processor object
   */
  ~SampleVendorRectifyProcessor() override;

  /**
   * @brief Rectify an image using camera calibration data with hardware acceleration
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
   * @brief Initialize capabilities specific to vendor hardware
   */
  void initialize_capabilities();

  /**
   * @brief Initialize the hardware accelerator
   * 
   * @return true if successful, false otherwise
   */
  bool initialize_hardware();

  /**
   * @brief Check if the hardware can handle the given image
   * 
   * @param image The image to check
   * @return true if hardware compatible, false otherwise
   */
  bool is_hardware_compatible(const sensor_msgs::msg::Image & image) const;

  /**
   * @brief Simulates hardware-accelerated rectification
   * 
   * In a real implementation, this would use the vendor's hardware API
   * 
   * @param input_image The input image
   * @param camera_info The camera calibration
   * @param output_image The output image
   * @return true if successful, false otherwise
   */
  bool rectify_with_hardware(
    const sensor_msgs::msg::Image & input_image,
    const sensor_msgs::msg::CameraInfo & camera_info,
    sensor_msgs::msg::Image & output_image);

  // In a real implementation, this would be a pointer to the vendor's hardware API
  void * hw_accelerator_;

  // Fallback OpenCV implementation when hardware acceleration fails
  std::shared_ptr<image_proc_hal::opencv::OpenCVRectifyProcessor> opencv_fallback_;

  // Hardware availability flag
  bool hw_available_;
};

}  // namespace sample_vendor
}  // namespace image_proc_hal

#endif  // IMAGE_PROC_HAL__PLUGINS__SAMPLE_VENDOR_RECTIFY_PROCESSOR_HPP_