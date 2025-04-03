// Copyright (c) 2023 ROS 2 Image Processing HAL Contributors
// Licensed under the Apache License, Version 2.0

#ifndef IMAGE_PROC_HAL__CAPABILITIES_HPP_
#define IMAGE_PROC_HAL__CAPABILITIES_HPP_

#include <string>
#include <vector>
#include <utility>

#include "image_proc_hal/visibility_control.hpp"
#include "image_proc_hal/enums.hpp"

namespace image_proc_hal
{

/**
 * @brief Class for declaring processor capabilities and limitations
 * 
 * This class allows processors to report what color spaces they support,
 * what resolution ranges they can handle, and other capabilities.
 */
class IMAGE_PROC_HAL_PUBLIC ImageProcessorCapabilities
{
public:
  /**
   * @brief Construct a new Image Processor Capabilities object
   * 
   * @param vendor_name The name of the vendor
   * @param implementation_name The name of the implementation
   */
  ImageProcessorCapabilities(
    const std::string & vendor_name = "default",
    const std::string & implementation_name = "default");

  /**
   * @brief Check if a specific color space is supported
   * 
   * @param color_space The color space to check
   * @return true if supported, false otherwise
   */
  bool supports_color_space(ColorSpace color_space) const;

  /**
   * @brief Check if a specific resolution is supported
   * 
   * @param width Image width in pixels
   * @param height Image height in pixels
   * @return true if supported, false otherwise
   */
  bool supports_resolution(uint32_t width, uint32_t height) const;

  /**
   * @brief Check if hardware acceleration is supported
   * 
   * @return true if hardware acceleration is supported, false otherwise
   */
  bool supports_hardware_acceleration() const;

  /**
   * @brief Get the supported color spaces
   * 
   * @return std::vector<ColorSpace> List of supported color spaces
   */
  std::vector<ColorSpace> get_supported_color_spaces() const;

  /**
   * @brief Get the minimum supported resolution
   * 
   * @return std::pair<uint32_t, uint32_t> Pair of minimum width and height
   */
  std::pair<uint32_t, uint32_t> get_min_resolution() const;

  /**
   * @brief Get the maximum supported resolution
   * 
   * @return std::pair<uint32_t, uint32_t> Pair of maximum width and height
   */
  std::pair<uint32_t, uint32_t> get_max_resolution() const;

  /**
   * @brief Get the vendor name
   * 
   * @return std::string The vendor name
   */
  std::string get_vendor_name() const;

  /**
   * @brief Get the implementation name
   * 
   * @return std::string The implementation name
   */
  std::string get_implementation_name() const;

  /**
   * @brief Add a supported color space
   * 
   * @param color_space The color space to add
   */
  void add_supported_color_space(ColorSpace color_space);

  /**
   * @brief Set whether hardware acceleration is supported
   * 
   * @param supported true if hardware acceleration is supported
   */
  void set_hardware_acceleration_supported(bool supported);

  /**
   * @brief Set the minimum resolution supported
   * 
   * @param width Minimum width in pixels
   * @param height Minimum height in pixels
   */
  void set_min_resolution(uint32_t width, uint32_t height);

  /**
   * @brief Set the maximum resolution supported
   * 
   * @param width Maximum width in pixels
   * @param height Maximum height in pixels
   */
  void set_max_resolution(uint32_t width, uint32_t height);

private:
  std::string vendor_name_;
  std::string implementation_name_;
  std::vector<ColorSpace> supported_color_spaces_;
  bool supports_hw_accel_;
  std::pair<uint32_t, uint32_t> min_resolution_;
  std::pair<uint32_t, uint32_t> max_resolution_;
};

}  // namespace image_proc_hal

#endif  // IMAGE_PROC_HAL__CAPABILITIES_HPP_