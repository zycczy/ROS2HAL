// Copyright (c) 2023 ROS 2 Image Processing HAL Contributors
// Licensed under the Apache License, Version 2.0

#ifndef IMAGE_PROC_HAL__PROCESSOR_FACTORY_HPP_
#define IMAGE_PROC_HAL__PROCESSOR_FACTORY_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "image_proc_hal/visibility_control.hpp"
#include "image_proc_hal/processor_base.hpp"
#include "image_proc_hal/rectify_processor.hpp"
#include "image_proc_hal/debayer_processor.hpp"
#include "image_proc_hal/resize_processor.hpp"

namespace image_proc_hal
{

/**
 * @brief Factory class for creating image processor instances
 * 
 * This class manages plugin discovery and instantiation for different
 * image processor types. It uses pluginlib to load and manage vendor
 * implementations dynamically.
 */
class IMAGE_PROC_HAL_PUBLIC ProcessorFactory
{
public:
  /**
   * @brief Get a singleton instance of the ProcessorFactory
   * 
   * @return ProcessorFactory& Reference to the singleton instance
   */
  static ProcessorFactory & get_instance();

  /**
   * @brief Create a processor of the specified type
   * 
   * This method creates a processor instance of the requested type,
   * optionally from a specific vendor. If the vendor is not specified
   * or not available, the factory will choose the best available
   * implementation based on capabilities.
   * 
   * @tparam T The type of processor to create (must inherit from ImageProcessorBase)
   * @param preferred_vendor The preferred vendor (optional)
   * @return std::shared_ptr<T> Shared pointer to the created processor
   */
  template<typename T>
  std::shared_ptr<T> create_processor(const std::string & preferred_vendor = "")
  {
    static_assert(
      std::is_base_of<ImageProcessorBase, T>::value,
      "T must be derived from ImageProcessorBase");

    // Get the appropriate class loader for this processor type
    auto class_loader = get_class_loader<T>();
    if (!class_loader) {
      return nullptr;
    }

    // Get list of available classes
    std::vector<std::string> available_classes = class_loader->getDeclaredClasses();
    
    if (available_classes.empty()) {
      return nullptr;
    }

    // Try to find the preferred vendor if specified
    std::string selected_class;
    if (!preferred_vendor.empty()) {
      for (const auto & class_name : available_classes) {
        if (class_name.find(preferred_vendor) != std::string::npos) {
          selected_class = class_name;
          break;
        }
      }
    }

    // If preferred vendor not found, use the first available class
    if (selected_class.empty()) {
      // First try to find a hardware-accelerated implementation
      for (const auto & class_name : available_classes) {
        try {
          auto instance = class_loader->createSharedInstance(class_name);
          if (instance->get_capabilities().supports_hardware_acceleration()) {
            return instance;
          }
        } catch (const pluginlib::PluginlibException & ex) {
          // Skip this implementation if it fails to load
          continue;
        }
      }
      
      // Fallback to any available implementation (likely OpenCV)
      selected_class = available_classes[0];
    }
    
    try {
      return class_loader->createSharedInstance(selected_class);
    } catch (const pluginlib::PluginlibException & ex) {
      return nullptr;
    }
  }

  /**
   * @brief Get available implementations for a specific processor type
   * 
   * @tparam T The type of processor
   * @return std::vector<std::string> List of available implementation names
   */
  template<typename T>
  std::vector<std::string> get_available_implementations()
  {
    static_assert(
      std::is_base_of<ImageProcessorBase, T>::value,
      "T must be derived from ImageProcessorBase");
    
    auto class_loader = get_class_loader<T>();
    if (!class_loader) {
      return {};
    }
    
    return class_loader->getDeclaredClasses();
  }

private:
  ProcessorFactory();
  ~ProcessorFactory();
  
  // Prevent copy construction and assignment
  ProcessorFactory(const ProcessorFactory &) = delete;
  ProcessorFactory & operator=(const ProcessorFactory &) = delete;
  
  /**
   * @brief Get or create a class loader for a specific processor type
   * 
   * @tparam T The type of processor
   * @return std::shared_ptr<pluginlib::ClassLoader<T>> Shared pointer to the class loader
   */
  template<typename T>
  std::shared_ptr<pluginlib::ClassLoader<T>> get_class_loader()
  {
    static_assert(
      std::is_base_of<ImageProcessorBase, T>::value,
      "T must be derived from ImageProcessorBase");
    
    const std::string base_class_name = typeid(T).name();
    
    // Check if we already have a loader for this type
    auto it = class_loaders_.find(base_class_name);
    if (it != class_loaders_.end()) {
      return std::static_pointer_cast<pluginlib::ClassLoader<T>>(it->second);
    }
    
    // Create appropriate package and class names based on the processor type
    std::string package_name = "image_proc_hal";
    std::string plugin_base_class;
    
    if (typeid(T) == typeid(RectifyProcessor)) {
      plugin_base_class = "image_proc_hal::RectifyProcessor";
    } else if (typeid(T) == typeid(DebayerProcessor)) {
      plugin_base_class = "image_proc_hal::DebayerProcessor";
    } else if (typeid(T) == typeid(ResizeProcessor)) {
      plugin_base_class = "image_proc_hal::ResizeProcessor";
    } else {
      // Unsupported processor type
      return nullptr;
    }
    
    // Create and store the class loader
    auto loader = std::make_shared<pluginlib::ClassLoader<T>>(package_name, plugin_base_class);
    class_loaders_[base_class_name] = std::static_pointer_cast<void>(loader);
    
    return loader;
  }
  
  // Map to store class loaders for different processor types
  std::map<std::string, std::shared_ptr<void>> class_loaders_;
};

}  // namespace image_proc_hal

#endif  // IMAGE_PROC_HAL__PROCESSOR_FACTORY_HPP_