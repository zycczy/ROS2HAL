// Copyright (c) 2023 ROS 2 Image Processing HAL Contributors
// Licensed under the Apache License, Version 2.0

#ifndef IMAGE_PROC_HAL__RESIZE_PROCESSOR_HPP_
#define IMAGE_PROC_HAL__RESIZE_PROCESSOR_HPP_

#include <memory>

#include "sensor_msgs/msg/image.hpp"
#include "image_proc_hal/visibility_control.hpp"
#include "image_proc_hal/processor_base.hpp"
#include "image_proc_hal/enums.hpp"

namespace image_proc_hal
{

/**
 * @brief Abstract interface for image resizing processors
 * 
 * This class defines the interface for resizing images.
 * Implementations of this interface can be provided by
 * different vendors to utilize hardware acceleration.
 */
class IMAGE_PROC_HAL_PUBLIC ResizeProcessor : public ImageProcessorBase
{
public:
  /**
   * @brief Destroy the Resize Processor object
   */
  virtual ~ResizeProcessor() = default;

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
  virtual bool resize(
    const sensor_msgs::msg::Image & input_image,
    sensor_msgs::msg::Image & output_image,
    uint32_t width, 
    uint32_t height,
    InterpolationMethod method = InterpolationMethod::LINEAR) = 0;
};

}  // namespace image_proc_hal

#endif  // IMAGE_PROC_HAL__RESIZE_PROCESSOR_HPP_