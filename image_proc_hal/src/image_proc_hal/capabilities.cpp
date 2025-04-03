// src/image_proc_hal/capabilities.cpp
#include "image_proc_hal/capabilities.hpp"

#include <algorithm>
#include <limits>

namespace image_proc_hal
{

ImageProcessorCapabilities::ImageProcessorCapabilities(
  const std::string & vendor_name,
  const std::string & implementation_name)
: vendor_name_(vendor_name),
  implementation_name_(implementation_name),
  supports_hw_accel_(false),
  min_resolution_({1, 1}),
  max_resolution_({std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max()})
{
  // Default initialization with RGB and BGR color spaces supported
  supported_color_spaces_.push_back(ColorSpace::RGB);
  supported_color_spaces_.push_back(ColorSpace::BGR);
  supported_color_spaces_.push_back(ColorSpace::GRAY);
}

bool ImageProcessorCapabilities::supports_color_space(ColorSpace color_space) const
{
  return std::find(
    supported_color_spaces_.begin(), 
    supported_color_spaces_.end(), 
    color_space) != supported_color_spaces_.end();
}

bool ImageProcessorCapabilities::supports_resolution(uint32_t width, uint32_t height) const
{
  return width >= min_resolution_.first && 
         height >= min_resolution_.second && 
         width <= max_resolution_.first && 
         height <= max_resolution_.second;
}

bool ImageProcessorCapabilities::supports_hardware_acceleration() const
{
  return supports_hw_accel_;
}

std::vector<ColorSpace> ImageProcessorCapabilities::get_supported_color_spaces() const
{
  return supported_color_spaces_;
}

std::pair<uint32_t, uint32_t> ImageProcessorCapabilities::get_min_resolution() const
{
  return min_resolution_;
}

std::pair<uint32_t, uint32_t> ImageProcessorCapabilities::get_max_resolution() const
{
  return max_resolution_;
}

std::string ImageProcessorCapabilities::get_vendor_name() const
{
  return vendor_name_;
}

std::string ImageProcessorCapabilities::get_implementation_name() const
{
  return implementation_name_;
}

void ImageProcessorCapabilities::add_supported_color_space(ColorSpace color_space)
{
  if (!supports_color_space(color_space)) {
    supported_color_spaces_.push_back(color_space);
  }
}

void ImageProcessorCapabilities::set_hardware_acceleration_supported(bool supported)
{
  supports_hw_accel_ = supported;
}

void ImageProcessorCapabilities::set_min_resolution(uint32_t width, uint32_t height)
{
  min_resolution_ = std::make_pair(width, height);
}

void ImageProcessorCapabilities::set_max_resolution(uint32_t width, uint32_t height)
{
  max_resolution_ = std::make_pair(width, height);
}

}  // namespace image_proc_hal