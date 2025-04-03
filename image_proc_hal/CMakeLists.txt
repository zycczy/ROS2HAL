cmake_minimum_required(VERSION 3.8)
project(image_proc_hal)

# Set C++ standard to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_geometry REQUIRED)
find_package(class_loader REQUIRED)
find_package(OpenCV REQUIRED)

# Define the include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Core library containing the interfaces
add_library(${PROJECT_NAME} SHARED
  src/image_proc_hal/capabilities.cpp
  src/image_proc_hal/processor_base.cpp
  src/image_proc_hal/rectify_processor.cpp
  src/image_proc_hal/debayer_processor.cpp
  src/image_proc_hal/resize_processor.cpp
  src/image_proc_hal/memory_manager.cpp
  src/image_proc_hal/processor_factory.cpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "IMAGE_PROC_HAL_BUILDING_DLL")
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  sensor_msgs
  pluginlib
  image_transport
  cv_bridge
  camera_info_manager
  image_geometry
  class_loader
  OpenCV
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# OpenCV default implementation
add_library(${PROJECT_NAME}_opencv SHARED
  src/image_proc_hal/plugins/opencv_rectify_processor.cpp
  src/image_proc_hal/plugins/opencv_debayer_processor.cpp
  src/image_proc_hal/plugins/opencv_resize_processor.cpp
  src/image_proc_hal/plugins/opencv_memory_manager.cpp
)
target_link_libraries(${PROJECT_NAME}_opencv ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_opencv
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Sample vendor implementation
add_library(${PROJECT_NAME}_sample_vendor SHARED
  src/image_proc_hal/plugins/sample_vendor_rectify_processor.cpp
)
target_link_libraries(${PROJECT_NAME}_sample_vendor ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_sample_vendor
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_image_proc_hal test/test_image_proc_hal.cpp)
  target_link_libraries(test_image_proc_hal ${PROJECT_NAME})
endif()

# Install
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_opencv ${PROJECT_NAME}_sample_vendor
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Install plugin description files for pluginlib
install(FILES plugins/opencv_plugins.xml plugins/sample_vendor_plugins.xml
  DESTINATION share/${PROJECT_NAME}/plugins
)

# Export package dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_opencv ${PROJECT_NAME}_sample_vendor)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp 
  sensor_msgs
  pluginlib
  cv_bridge
  OpenCV
)

ament_package()