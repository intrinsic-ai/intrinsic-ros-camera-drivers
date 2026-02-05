cmake_minimum_required(VERSION 3.8)
project(flowstate_common)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-g -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)

find_package(abseil_cpp_vendor REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(snapshot_interfaces REQUIRED)

include_directories(include)

# Flowstate common library
add_library(flowstate_common
  src/camera_adapter_node.cc
  src/camera_spawner_node.cc
  src/image_utils.cc
)

target_link_libraries(flowstate_common
  absl::status
  absl::synchronization
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${snapshot_interfaces_TARGETS}
)

ament_target_dependencies(flowstate_common
  cv_bridge
  OpenCV
)

install(TARGETS flowstate_common
        DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include)

ament_export_targets(flowstate_common HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  snapshot_interfaces
  absl_cpp
  OpenCV
  cv_bridge
)

ament_package()
