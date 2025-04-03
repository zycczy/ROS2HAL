// src/image_proc_hal/plugins/opencv_memory_manager.cpp
#include "src/image_proc_hal/plugins/opencv_memory_manager.hpp"

#include <cstring>  // for memcpy
#include <cstdint>  // for uintptr_t
#include <memory>   // for std::align
#include <new>      // for std::bad_alloc

namespace image_proc_hal
{
namespace opencv
{

void * OpenCVMemoryManager::allocate_aligned_memory(size_t size, size_t alignment)
{
  // Check for invalid input
  if (size == 0 || alignment == 0 || (alignment & (alignment - 1)) != 0) {
    // Alignment must be a power of 2 and size must be positive
    return nullptr;
  }

  // Allocate with standard C++17 aligned_alloc if available
#if defined(_ISOC11_SOURCE) || (defined(__cplusplus) && __cplusplus >= 201703L)
  try {
    return std::aligned_alloc(alignment, size);
  } catch (const std::bad_alloc &) {
    return nullptr;
  }
#else
  // Fallback implementation using malloc with extra space
  void * raw_ptr = malloc(size + alignment);
  if (raw_ptr == nullptr) {
    return nullptr;
  }

  // Calculate aligned pointer
  uintptr_t address = reinterpret_cast<uintptr_t>(raw_ptr);
  uintptr_t aligned_address = (address + alignment) & ~(alignment - 1);
  void * aligned_ptr = reinterpret_cast<void *>(aligned_address);
  
  // Store original pointer before the aligned memory
  *(reinterpret_cast<void **>(aligned_address) - 1) = raw_ptr;
  
  return aligned_ptr;
#endif
}

void OpenCVMemoryManager::free_aligned_memory(void * ptr)
{
  if (ptr == nullptr) {
    return;
  }

#if defined(_ISOC11_SOURCE) || (defined(__cplusplus) && __cplusplus >= 201703L)
  // Use standard free for std::aligned_alloc
  free(ptr);
#else
  // Get original pointer for our custom aligned allocation
  void * raw_ptr = *(reinterpret_cast<void **>(ptr) - 1);
  free(raw_ptr);
#endif
}

bool OpenCVMemoryManager::copy_to_device(void * device_ptr, const void * host_ptr, size_t size)
{
  if (device_ptr == nullptr || host_ptr == nullptr || size == 0) {
    return false;
  }
  
  // In the CPU-only OpenCV implementation, both pointers are in host memory
  // so we can just use a standard memcpy
  std::memcpy(device_ptr, host_ptr, size);
  return true;
}

bool OpenCVMemoryManager::copy_from_device(void * host_ptr, const void * device_ptr, size_t size)
{
  if (host_ptr == nullptr || device_ptr == nullptr || size == 0) {
    return false;
  }
  
  // In the CPU-only OpenCV implementation, both pointers are in host memory
  // so we can just use a standard memcpy
  std::memcpy(host_ptr, device_ptr, size);
  return true;
}

bool OpenCVMemoryManager::supports_zero_copy()
{
  // OpenCV implementation doesn't have separate device memory,
  // so zero-copy is inherently supported
  return true;
}

void * OpenCVMemoryManager::get_device_accessible_pointer(void * host_ptr)
{
  // In OpenCV implementation, the host pointer is already accessible to the "device" (CPU)
  return host_ptr;
}

}  // namespace opencv
}  // namespace image_proc_hal