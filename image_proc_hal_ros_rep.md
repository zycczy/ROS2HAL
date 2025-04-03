# REP: Hardware Abstraction Layer (HAL) for ROS 2 Image Processing

| REP           | XX                                       |
|---------------|------------------------------------------|
| Title         | Hardware Abstraction Layer (HAL) for ROS 2 Image Processing |
| Author(s)     | *Your Name* |
| Status        | Draft                                  |
| Type          | Standards Track                          |
| ROS Version   | Humble, Iron, Rolling                    |
| Created       | 2025-03-28                               |

## Abstract

This REP proposes a Hardware Abstraction Layer (HAL) for image processing in ROS 2 that enables leveraging hardware acceleration capabilities across different vendor platforms while maintaining compatibility with the existing ROS 2 image processing ecosystem. The HAL provides a plugin-based architecture that allows chipset vendors to implement optimized versions of common image processing operations without requiring changes to application code.

## Motivation

As robotics systems increasingly rely on compute-intensive vision applications, the ability to leverage hardware acceleration becomes critical for performance, power efficiency, and real-time operation. Modern System-on-Chips (SoCs) and specialized hardware often include dedicated image processing units (IPUs), neural processing units (NPUs), GPUs, or other accelerators that can significantly outperform CPU-based implementations.

However, the current ROS 2 image processing stack lacks a standardized way for hardware vendors to provide optimized implementations while preserving compatibility with existing codebases. This leads to several issues:

1. **Fragmentation**: Vendors create custom, non-portable solutions that lock users into specific hardware platforms
2. **Code duplication**: Developers implement similar optimizations repeatedly across projects
3. **Lack of fallbacks**: Hardware-specific implementations often don't gracefully degrade to software alternatives when specialized hardware is unavailable
4. **Maintenance burden**: Supporting multiple hardware platforms requires significant effort

This REP addresses these issues by defining a common interface and plugin architecture that enables vendors to provide hardware-accelerated image processing while maintaining compatibility with the existing ROS 2 ecosystem.

## Rationale

The design proposed in this REP aims to balance several competing objectives:

1. **Performance**: Enable optimal use of hardware acceleration capabilities
2. **Compatibility**: Maintain compatibility with existing ROS 2 image processing frameworks
3. **Flexibility**: Support diverse hardware platforms and capabilities
4. **Usability**: Provide a simple interface for both users and vendors
5. **Robustness**: Ensure reliable operation with fallback mechanisms

A plugin-based architecture was chosen because it allows dynamic loading of vendor-specific implementations at runtime, based on availability and capabilities. This approach has several advantages:

- It avoids compile-time dependencies on specific hardware platforms
- It enables automatic fallback to software implementations when hardware acceleration is unavailable
- It provides a clear separation between the interface and implementation
- It aligns with existing ROS 2 plugin systems

The HAL focuses on common image processing operations (rectification, debayering, and resizing) that are frequently used in robotics applications and benefit significantly from hardware acceleration. These operations were selected based on their computational intensity, frequency of use, and suitability for acceleration across diverse hardware platforms.

## Specification

### Architecture Overview

The Hardware Abstraction Layer consists of several key components:

1. **Core HAL Interfaces**: Abstract base classes defining the API for various image processing operations
2. **Plugin Architecture**: A system for dynamically loading vendor-specific implementations
3. **Default Implementations**: Fallback implementations using standard libraries like OpenCV
4. **Memory Management**: Abstractions for efficient memory handling and zero-copy optimizations
5. **Capability Reporting**: Mechanisms for discovering and selecting the most appropriate implementation

### Core HAL Interfaces

The HAL defines abstract interfaces for common image processing operations:

#### Base Processor Interface

All specific processor types inherit from a common base class (`ImageProcessorBase`) that provides capability reporting and compatibility checking:

```cpp
class ImageProcessorBase
{
public:
  ImageProcessorBase();
  virtual ~ImageProcessorBase() = default;
  
  // Get the capabilities of this processor
  ImageProcessorCapabilities get_capabilities() const;
  
  // Check if this processor is compatible with a given image
  bool is_compatible(const sensor_msgs::msg::Image & image) const;

protected:
  ImageProcessorCapabilities capabilities_;
};
```

#### Specific Processor Interfaces

Specialized interfaces are defined for specific image processing operations:

**Rectification Processor**:
```cpp
class RectifyProcessor : public ImageProcessorBase
{
public:
  virtual ~RectifyProcessor() = default;
  
  // Rectify an image using camera calibration data
  virtual bool rectify(
    const sensor_msgs::msg::Image & input_image,
    const sensor_msgs::msg::CameraInfo & camera_info, 
    sensor_msgs::msg::Image & output_image) = 0;
};
```

**Debayer Processor**:
```cpp
class DebayerProcessor : public ImageProcessorBase
{
public:
  virtual ~DebayerProcessor() = default;
  
  // Convert a Bayer pattern image to a color image
  virtual bool debayer(
    const sensor_msgs::msg::Image & bayer_image,
    sensor_msgs::msg::Image & color_image,
    DebayerAlgorithm algorithm = DebayerAlgorithm::BILINEAR) = 0;
};
```

**Resize Processor**:
```cpp
class ResizeProcessor : public ImageProcessorBase
{
public:
  virtual ~ResizeProcessor() = default;
  
  // Resize an image
  virtual bool resize(
    const sensor_msgs::msg::Image & input_image,
    sensor_msgs::msg::Image & output_image,
    uint32_t width, 
    uint32_t height,
    InterpolationMethod method = InterpolationMethod::LINEAR) = 0;
};
```

### Capabilities Reporting

The HAL includes a capabilities system that allows processors to advertise their supported features and limitations:

```cpp
class ImageProcessorCapabilities
{
public:
  ImageProcessorCapabilities(
    const std::string & vendor_name = "default",
    const std::string & implementation_name = "default");
  
  // Check for specific capabilities
  bool supports_color_space(ColorSpace color_space) const;
  bool supports_resolution(uint32_t width, uint32_t height) const;
  bool supports_hardware_acceleration() const;
  
  // Get capability information
  std::vector<ColorSpace> get_supported_color_spaces() const;
  std::pair<uint32_t, uint32_t> get_min_resolution() const;
  std::pair<uint32_t, uint32_t> get_max_resolution() const;
  std::string get_vendor_name() const;
  std::string get_implementation_name() const;
  
  // Set capability information
  void add_supported_color_space(ColorSpace color_space);
  void set_hardware_acceleration_supported(bool supported);
  void set_min_resolution(uint32_t width, uint32_t height);
  void set_max_resolution(uint32_t width, uint32_t height);
  
  // ... other capability methods
};
```

### Plugin Architecture

The HAL uses the ROS 2 pluginlib system to enable dynamic loading of vendor-specific implementations:

```cpp
class ProcessorFactory
{
public:
  static ProcessorFactory & get_instance();
  
  // Create a processor of the specified type
  template<typename T>
  std::shared_ptr<T> create_processor(const std::string & preferred_vendor = "");
  
  // Get available implementations for a specific processor type
  template<typename T>
  std::vector<std::string> get_available_implementations();
  
  // ... other factory methods
};
```

Vendors register their implementations using XML plugin description files:

```xml
<!-- Example: opencv_plugins.xml -->
<library path="opencv_image_proc_plugins">
  <class name="opencv/RectifyProcessor" type="opencv_image_proc::OpenCVRectifyProcessor" base_class_type="image_proc_hal::RectifyProcessor">
    <description>OpenCV implementation of image rectification</description>
  </class>
  <!-- Other processor implementations -->
</library>
```

### Memory Management

The HAL includes memory management abstractions to enable efficient data transfer between CPU and hardware accelerators:

```cpp
class MemoryManager
{
public:
  virtual ~MemoryManager() = default;
  
  // Memory allocation and deallocation
  virtual void * allocate_aligned_memory(size_t size, size_t alignment) = 0;
  virtual void free_aligned_memory(void * ptr) = 0;
  
  // Data transfer between host and device
  virtual bool copy_to_device(void * device_ptr, const void * host_ptr, size_t size) = 0;
  virtual bool copy_from_device(void * host_ptr, const void * device_ptr, size_t size) = 0;
  
  // Zero-copy interface
  virtual bool supports_zero_copy() = 0;
  virtual void * get_device_accessible_pointer(void * host_ptr) = 0;
};
```

## Implementation

### Default Implementations

The HAL includes default implementations based on OpenCV for all processor types. These implementations serve as fallbacks when hardware-accelerated versions are unavailable or incompatible with the input data.

### Vendor Implementation Guidelines

Vendors implementing the HAL interfaces should follow these guidelines:

1. **Derive from base interfaces**: Implement the appropriate abstract base class for each processor type
2. **Report capabilities accurately**: Properly initialize the capabilities object to reflect supported features
3. **Handle errors gracefully**: Return appropriate error codes when operations fail
4. **Optimize for common cases**: Focus optimization efforts on the most common use cases
5. **Provide memory management**: Implement the MemoryManager interface if custom memory handling is required

### Integration with Existing Image Processing Pipelines

The HAL can be integrated into existing image processing pipelines in several ways:

1. **Direct API usage**: Applications can directly create and use processor instances via the ProcessorFactory
2. **Node integration**: ROS 2 nodes can be implemented that use the HAL internally
3. **Composable nodes**: Composable node implementations allow for zero-copy communication in process containers

## Compatibility

The HAL is designed to be compatible with the existing ROS 2 image processing ecosystem, including:

- **sensor_msgs**: Uses standard ROS 2 message types like sensor_msgs/msg/Image and sensor_msgs/msg/CameraInfo
- **image_transport**: Can be used alongside image_transport for efficient image publishing and subscription
- **image_pipeline**: Can replace or augment components in the standard image_pipeline stack

## References

1. ROS 2 image_pipeline: https://github.com/ros-perception/image_pipeline
2. ROS 2 pluginlib: https://github.com/ros/pluginlib
3. OpenCV: https://opencv.org/

## Copyright

This document has been placed in the public domain.
