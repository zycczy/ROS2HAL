// Copyright (c) 2023 ROS 2 Image Processing HAL Contributors
// Licensed under the Apache License, Version 2.0

#ifndef IMAGE_PROC_HAL__PROCESSOR_BASE_HPP_
#define IMAGE_PROC_HAL__PROCESSOR_BASE_HPP_

#include <memory>

#include "sensor_msgs/msg/image.hpp"
#include "image_proc_hal/visibility_control.hpp"
#include "image_proc_hal/capabilities.hpp"

namespace image_proc_hal
{

/**
 * @brief Base class for all image processors
 * 
 * This class defines the common interface and functionality for all image processors.
 * It includes capability reporting and compatibility checking.
 */
class IMAGE_PROC_HAL_PUBLIC ImageProcessorBase
{
public:
  /**
   * @brief Construct a new Image Processor Base object
   */
  ImageProcessorBase();

  /**
   * @brief Destroy the Image Processor Base object
   */
  virtual ~ImageProcessorBase() = default;

  /**
   * @brief Get the capabilities of this processor
   * 
   * @return The processor's capabilities
   */
  ImageProcessorCapabilities get_capabilities() const;

  /**
   * @brief Check if this processor is compatible with a given image
   * 
   * @param image The image to check compatibility with
   * @return true if compatible, false otherwise
   */
  bool is_compatible(const sensor_msgs::msg::Image & image) const;

protected:
  /**
   * @brief The processor's capabilities
   */
  ImageProcessorCapabilities capabilities_;
};

}  // namespace image_proc_hal

#endif  // IMAGE_PROC_HAL__PROCESSOR_BASE_HPP_