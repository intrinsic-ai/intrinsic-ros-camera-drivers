cmake_minimum_required(VERSION 3.8)
project(flowstate_camera_common)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-g -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)

find_package(abseil_cpp_vendor REQUIRED)
find_package(absl REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(snapshot_interfaces REQUIRED)

include_directories(include)

add_library(flowstate_camera_common SHARED
  src/base_adapter_node.cc
  src/base_spawner_node.cc
)
target_link_libraries(flowstate_camera_common
  absl::status
  absl::synchronization
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${snapshot_interfaces_TARGETS}
)
ament_target_dependencies(flowstate_camera_common
)
install(TARGETS flowstate_camera_common
        DESTINATION lib)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
