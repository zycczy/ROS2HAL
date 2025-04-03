// Copyright (c) 2023 ROS 2 Image Processing HAL Contributors
// Licensed under the Apache License, Version 2.0

#ifndef IMAGE_PROC_HAL__ENUMS_HPP_
#define IMAGE_PROC_HAL__ENUMS_HPP_

#include <string>

#include "image_proc_hal/visibility_control.hpp"

namespace image_proc_hal
{

/**
 * @brief Enumeration for supported color spaces
 */
enum class ColorSpace
{
  RGB,        ///< Red, Green, Blue color space
  BGR,        ///< Blue, Green, Red color space (OpenCV default)
  GRAY,       ///< Single channel grayscale
  BAYER_RGGB, ///< Bayer pattern with RGGB layout
  BAYER_BGGR, ///< Bayer pattern with BGGR layout
  BAYER_GBRG, ///< Bayer pattern with GBRG layout
  BAYER_GRBG, ///< Bayer pattern with GRBG layout
  YUV,        ///< YUV color space
  YUV422      ///< YUV422 color space (subsampled)
};

/**
 * @brief Enumeration for debayering algorithms
 */
enum class DebayerAlgorithm
{
  NEAREST,    ///< Nearest neighbor interpolation
  BILINEAR,   ///< Bilinear interpolation
  HQLINEAR,   ///< High-quality linear interpolation
  EDGE_AWARE, ///< Edge-aware interpolation
  VNG         ///< Variable Number of Gradients
};

/**
 * @brief Enumeration for interpolation methods
 */
enum class InterpolationMethod
{
  NEAREST, ///< Nearest neighbor interpolation
  LINEAR,  ///< Linear interpolation
  CUBIC,   ///< Cubic interpolation
  LANCZOS  ///< Lanczos interpolation
};

/**
 * @brief Convert ColorSpace enum to string
 *
 * @param color_space The ColorSpace enum value
 * @return std::string The string representation
 */
IMAGE_PROC_HAL_PUBLIC
std::string to_string(ColorSpace color_space);

/**
 * @brief Convert DebayerAlgorithm enum to string
 *
 * @param algorithm The DebayerAlgorithm enum value
 * @return std::string The string representation
 */
IMAGE_PROC_HAL_PUBLIC
std::string to_string(DebayerAlgorithm algorithm);

/**
 * @brief Convert InterpolationMethod enum to string
 *
 * @param method The InterpolationMethod enum value
 * @return std::string The string representation
 */
IMAGE_PROC_HAL_PUBLIC
std::string to_string(InterpolationMethod method);

}  // namespace image_proc_hal

#endif  // IMAGE_PROC_HAL__ENUMS_HPP_