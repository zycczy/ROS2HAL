// Copyright (c) 2023 ROS 2 Image Processing HAL Contributors
// Licensed under the Apache License, Version 2.0

#ifndef IMAGE_PROC_HAL__RECTIFY_PROCESSOR_HPP_
#define IMAGE_PROC_HAL__RECTIFY_PROCESSOR_HPP_

#include <memory>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_proc_hal/visibility_control.hpp"
#include "image_proc_hal/processor_base.hpp"

namespace image_proc_hal
{

/**
 * @brief Abstract interface for image rectification processors
 * 
 * This class defines the interface for rectifying images using
 * camera calibration information. Implementations of this interface
 * can be provided by different vendors to utilize hardware acceleration.
 */
class IMAGE_PROC_HAL_PUBLIC RectifyProcessor : public ImageProcessorBase
{
public:
  /**
   * @brief Destroy the Rectify Processor object
   */
  virtual ~RectifyProcessor() = default;

  /**
   * @brief Rectify an image using camera calibration data
   * 
   * @param input_image The distorted input image
   * @param camera_info Camera calibration information
   * @param output_image The rectified output image
   * @return true if successful, false otherwise
   */
  virtual bool rectify(
    const sensor_msgs::msg::Image & input_image,
    const sensor_msgs::msg::CameraInfo & camera_info, 
    sensor_msgs::msg::Image & output_image) = 0;
};

}  // namespace image_proc_hal

#endif  // IMAGE_PROC_HAL__RECTIFY_PROCESSOR_HPP_