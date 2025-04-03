// Copyright (c) 2023 ROS 2 Image Processing HAL Contributors
// Licensed under the Apache License, Version 2.0

#ifndef IMAGE_PROC_HAL__DEBAYER_PROCESSOR_HPP_
#define IMAGE_PROC_HAL__DEBAYER_PROCESSOR_HPP_

#include <memory>

#include "sensor_msgs/msg/image.hpp"
#include "image_proc_hal/visibility_control.hpp"
#include "image_proc_hal/processor_base.hpp"
#include "image_proc_hal/enums.hpp"

namespace image_proc_hal
{

/**
 * @brief Abstract interface for debayering (demosaicing) processors
 * 
 * This class defines the interface for converting Bayer pattern images
 * to color images. Implementations of this interface can be provided
 * by different vendors to utilize hardware acceleration.
 */
class IMAGE_PROC_HAL_PUBLIC DebayerProcessor : public ImageProcessorBase
{
public:
  /**
   * @brief Destroy the Debayer Processor object
   */
  virtual ~DebayerProcessor() = default;

  /**
   * @brief Convert a Bayer pattern image to a color image
   * 
   * @param bayer_image The Bayer pattern input image
   * @param color_image The color output image
   * @param algorithm The debayering algorithm to use
   * @return true if successful, false otherwise
   */
  virtual bool debayer(
    const sensor_msgs::msg::Image & bayer_image,
    sensor_msgs::msg::Image & color_image,
    DebayerAlgorithm algorithm = DebayerAlgorithm::BILINEAR) = 0;
};

}  // namespace image_proc_hal

#endif  // IMAGE_PROC_HAL__DEBAYER_PROCESSOR_HPP_