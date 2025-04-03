// Copyright (c) 2023 ROS 2 Image Processing HAL Contributors
// Licensed under the Apache License, Version 2.0

#ifndef IMAGE_PROC_HAL__MEMORY_MANAGER_HPP_
#define IMAGE_PROC_HAL__MEMORY_MANAGER_HPP_

#include <cstddef>  // for size_t

#include "image_proc_hal/visibility_control.hpp"

namespace image_proc_hal
{

/**
 * @brief Abstract interface for memory management
 * 
 * This class defines the interface for hardware-specific memory management
 * operations. It allows for efficient memory allocation, transfer, and
 * potential zero-copy optimizations between the CPU and hardware accelerators.
 */
class IMAGE_PROC_HAL_PUBLIC MemoryManager
{
public:
  /**
   * @brief Destroy the Memory Manager object
   */
  virtual ~MemoryManager() = default;

  /**
   * @brief Allocate aligned memory suitable for hardware acceleration
   * 
   * @param size The size in bytes to allocate
   * @param alignment The memory alignment in bytes (typically 16, 32, 64, or 128)
   * @return void* Pointer to the allocated memory, or nullptr on failure
   */
  virtual void * allocate_aligned_memory(size_t size, size_t alignment) = 0;

  /**
   * @brief Free memory allocated with allocate_aligned_memory
   * 
   * @param ptr Pointer to the memory to free
   */
  virtual void free_aligned_memory(void * ptr) = 0;

  /**
   * @brief Copy data from host memory to device memory
   * 
   * @param device_ptr Destination pointer in device memory
   * @param host_ptr Source pointer in host memory
   * @param size Number of bytes to copy
   * @return true if successful, false otherwise
   */
  virtual bool copy_to_device(void * device_ptr, const void * host_ptr, size_t size) = 0;

  /**
   * @brief Copy data from device memory to host memory
   * 
   * @param host_ptr Destination pointer in host memory
   * @param device_ptr Source pointer in device memory
   * @param size Number of bytes to copy
   * @return true if successful, false otherwise
   */
  virtual bool copy_from_device(void * host_ptr, const void * device_ptr, size_t size) = 0;

  /**
   * @brief Check if zero-copy transfers are supported
   * 
   * @return true if zero-copy is supported, false otherwise
   */
  virtual bool supports_zero_copy() = 0;

  /**
   * @brief Get a device-accessible pointer from a host pointer
   * 
   * If the implementation supports zero-copy, this method returns a pointer
   * that can be accessed by the device without explicit copies.
   * 
   * @param host_ptr The host pointer to convert
   * @return void* The device-accessible pointer, or nullptr if not supported
   */
  virtual void * get_device_accessible_pointer(void * host_ptr) = 0;
};

}  // namespace image_proc_hal

#endif  // IMAGE_PROC_HAL__MEMORY_MANAGER_HPP_