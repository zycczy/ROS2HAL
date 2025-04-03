// src/image_proc_hal/processor_factory.cpp
#include "image_proc_hal/processor_factory.hpp"
#include "rclcpp/rclcpp.hpp"

namespace image_proc_hal
{

ProcessorFactory & ProcessorFactory::get_instance()
{
  static ProcessorFactory instance;
  return instance;
}

ProcessorFactory::ProcessorFactory()
{
  // Initialize the class loaders
}

ProcessorFactory::~ProcessorFactory()
{
  // Clean up class loaders
  class_loaders_.clear();
}

}  // namespace image_proc_hal