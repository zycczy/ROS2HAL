// src/image_proc_hal/plugins/opencv_memory_manager.hpp
#ifndef IMAGE_PROC_HAL__PLUGINS__OPENCV_MEMORY_MANAGER_HPP_
#define IMAGE_PROC_HAL__PLUGINS__OPENCV_MEMORY_MANAGER_HPP_

#include "image_proc_hal/memory_manager.hpp"

namespace image_proc_hal
{
namespace opencv
{

/**
 * @brief OpenCV-based implementation of the memory manager
 * 
 * This class provides a basic implementation of the memory manager
 * interface using standard C++ memory functions. This implementation
 * doesn't provide hardware acceleration but serves as a fallback.
 */
class OpenCVMemoryManager : public MemoryManager
{
public:
  /**
   * @brief Construct a new OpenCV Memory Manager object
   */
  OpenCVMemoryManager() = default;

  /**
   * @brief Destroy the OpenCV Memory Manager object
   */
  ~OpenCVMemoryManager() override = default;

  /**
   * @brief Allocate aligned memory
   * 
   * @param size The size in bytes to allocate
   * @param alignment The memory alignment in bytes
   * @return void* Pointer to the allocated memory, or nullptr on failure
   */
  void * allocate_aligned_memory(size_t size, size_t alignment) override;

  /**
   * @brief Free aligned memory
   * 
   * @param ptr Pointer to the memory to free
   */
  void free_aligned_memory(void * ptr) override;

  /**
   * @brief Copy data from host memory to device memory
   * 
   * @param device_ptr Destination pointer in device memory
   * @param host_ptr Source pointer in host memory
   * @param size Number of bytes to copy
   * @return true if successful, false otherwise
   */
  bool copy_to_device(void * device_ptr, const void * host_ptr, size_t size) override;

  /**
   * @brief Copy data from device memory to host memory
   * 
   * @param host_ptr Destination pointer in host memory
   * @param device_ptr Source pointer in device memory
   * @param size Number of bytes to copy
   * @return true if successful, false otherwise
   */
  bool copy_from_device(void * host_ptr, const void * device_ptr, size_t size) override;

  /**
   * @brief Check if zero-copy transfers are supported
   * 
   * @return true if zero-copy is supported, false otherwise
   */
  bool supports_zero_copy() override;

  /**
   * @brief Get a device-accessible pointer from a host pointer
   * 
   * @param host_ptr The host pointer to convert
   * @return void* The device-accessible pointer, or nullptr if not supported
   */
  void * get_device_accessible_pointer(void * host_ptr) override;
};

}  // namespace opencv
}  // namespace image_proc_hal

#endif  // IMAGE_PROC_HAL__PLUGINS__OPENCV_MEMORY_MANAGER_HPP_