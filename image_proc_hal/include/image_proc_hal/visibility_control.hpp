// Copyright (c) 2023 ROS 2 Image Processing HAL Contributors
// Licensed under the Apache License, Version 2.0

#ifndef IMAGE_PROC_HAL__VISIBILITY_CONTROL_HPP_
#define IMAGE_PROC_HAL__VISIBILITY_CONTROL_HPP_

// This logic was borrowed from the examples in the ROS 2 documentation:
// https://docs.ros.org/en/rolling/Tutorials/Building-a-Custom-Plugin.html

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IMAGE_PROC_HAL_EXPORT __attribute__ ((dllexport))
    #define IMAGE_PROC_HAL_IMPORT __attribute__ ((dllimport))
  #else
    #define IMAGE_PROC_HAL_EXPORT __declspec(dllexport)
    #define IMAGE_PROC_HAL_IMPORT __declspec(dllimport)
  #endif
  #ifdef IMAGE_PROC_HAL_BUILDING_DLL
    #define IMAGE_PROC_HAL_PUBLIC IMAGE_PROC_HAL_EXPORT
  #else
    #define IMAGE_PROC_HAL_PUBLIC IMAGE_PROC_HAL_IMPORT
  #endif
  #define IMAGE_PROC_HAL_PUBLIC_TYPE IMAGE_PROC_HAL_PUBLIC
  #define IMAGE_PROC_HAL_LOCAL
#else
  #define IMAGE_PROC_HAL_EXPORT __attribute__ ((visibility("default")))
  #define IMAGE_PROC_HAL_IMPORT
  #if __GNUC__ >= 4
    #define IMAGE_PROC_HAL_PUBLIC __attribute__ ((visibility("default")))
    #define IMAGE_PROC_HAL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IMAGE_PROC_HAL_PUBLIC
    #define IMAGE_PROC_HAL_LOCAL
  #endif
  #define IMAGE_PROC_HAL_PUBLIC_TYPE
#endif

#endif  // IMAGE_PROC_HAL__VISIBILITY_CONTROL_HPP_