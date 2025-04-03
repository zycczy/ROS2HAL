// src/image_proc_hal/processor_base.cpp
#include "image_proc_hal/processor_base.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace image_proc_hal
{

ImageProcessorBase::ImageProcessorBase()
{
  // Initialize with default capabilities
}

ImageProcessorCapabilities ImageProcessorBase::get_capabilities() const
{
  return capabilities_;
}

bool ImageProcessorBase::is_compatible(const sensor_msgs::msg::Image & image) const
{
  // Check image dimensions
  if (!capabilities_.supports_resolution(image.width, image.height)) {
    return false;
  }

  // Determine color space from encoding
  ColorSpace color_space;
  const std::string & encoding = image.encoding;

  if (encoding == sensor_msgs::image_encodings::RGB8) {
    color_space = ColorSpace::RGB;
  } else if (encoding == sensor_msgs::image_encodings::BGR8) {
    color_space = ColorSpace::BGR;
  } else if (encoding == sensor_msgs::image_encodings::MONO8) {
    color_space = ColorSpace::GRAY;
  } else if (encoding == sensor_msgs::image_encodings::BAYER_RGGB8) {
    color_space = ColorSpace::BAYER_RGGB;
  } else if (encoding == sensor_msgs::image_encodings::BAYER_BGGR8) {
    color_space = ColorSpace::BAYER_BGGR;
  } else if (encoding == sensor_msgs::image_encodings::BAYER_GBRG8) {
    color_space = ColorSpace::BAYER_GBRG;
  } else if (encoding == sensor_msgs::image_encodings::BAYER_GRBG8) {
    color_space = ColorSpace::BAYER_GRBG;
  } else if (encoding.find("yuv") != std::string::npos) {
    if (encoding.find("422") != std::string::npos) {
      color_space = ColorSpace::YUV422;
    } else {
      color_space = ColorSpace::YUV;
    }
  } else {
    // Unsupported color space
    return false;
  }

  // Check if color space is supported
  return capabilities_.supports_color_space(color_space);
}

}  // namespace image_proc_hal