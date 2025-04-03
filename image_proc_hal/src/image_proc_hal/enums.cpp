// src/image_proc_hal/enums.cpp
#include "image_proc_hal/enums.hpp"

namespace image_proc_hal
{

std::string to_string(ColorSpace color_space)
{
  switch (color_space) {
    case ColorSpace::RGB:
      return "RGB";
    case ColorSpace::BGR:
      return "BGR";
    case ColorSpace::GRAY:
      return "GRAY";
    case ColorSpace::BAYER_RGGB:
      return "BAYER_RGGB";
    case ColorSpace::BAYER_BGGR:
      return "BAYER_BGGR";
    case ColorSpace::BAYER_GBRG:
      return "BAYER_GBRG";
    case ColorSpace::BAYER_GRBG:
      return "BAYER_GRBG";
    case ColorSpace::YUV:
      return "YUV";
    case ColorSpace::YUV422:
      return "YUV422";
    default:
      return "UNKNOWN";
  }
}

std::string to_string(DebayerAlgorithm algorithm)
{
  switch (algorithm) {
    case DebayerAlgorithm::NEAREST:
      return "NEAREST";
    case DebayerAlgorithm::BILINEAR:
      return "BILINEAR";
    case DebayerAlgorithm::HQLINEAR:
      return "HQLINEAR";
    case DebayerAlgorithm::EDGE_AWARE:
      return "EDGE_AWARE";
    case DebayerAlgorithm::VNG:
      return "VNG";
    default:
      return "UNKNOWN";
  }
}

std::string to_string(InterpolationMethod method)
{
  switch (method) {
    case InterpolationMethod::NEAREST:
      return "NEAREST";
    case InterpolationMethod::LINEAR:
      return "LINEAR";
    case InterpolationMethod::CUBIC:
      return "CUBIC";
    case InterpolationMethod::LANCZOS:
      return "LANCZOS";
    default:
      return "UNKNOWN";
  }
}

}  // namespace image_proc_hal