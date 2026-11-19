# Copyright 2026 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(flowstate_realsense)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-g -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)

find_package(abseil_cpp_vendor REQUIRED)
find_package(absl REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(flowstate_camera_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(realsense2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(snapshot_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(intrinsic_sdk_ros QUIET)

include_directories(include)

add_library(flowstate_realsense_helpers STATIC
  src/helpers.cc
)
target_link_libraries(flowstate_realsense_helpers
  absl::status
  absl::statusor
  absl::strings
  ${sensor_msgs_TARGETS}
)

# realsense2_camera itself is loaded at run time as the rclcpp component it
# registers (adapter_node.cc), so it is an exec dependency only: no vendor
# header is included and none of its dependencies are linked here.
add_library(flowstate_realsense
  src/adapter_node.cc
  src/spawner_node.cc
)
target_link_libraries(flowstate_realsense
  flowstate_realsense_helpers
  ${flowstate_camera_common_LIBRARIES}
  absl::algorithm_container
  absl::flat_hash_set
  absl::status
  absl::statusor
  absl::strings
  absl::synchronization
  absl::time
  ament_index_cpp::ament_index_cpp
  class_loader::class_loader
  rclcpp::rclcpp
  rclcpp_components::component
  realsense2::realsense2
  tf2::tf2
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${snapshot_interfaces_TARGETS}
)
ament_target_dependencies(flowstate_realsense
  flowstate_camera_common
)

add_executable(spawner_node src/ros_main.cc)
target_link_libraries(spawner_node flowstate_realsense)

install(TARGETS spawner_node
        DESTINATION lib/${PROJECT_NAME}
)

if (intrinsic_sdk_ros_FOUND)
  message(STATUS "intrinsic_sdk_ros found")
  intrinsic_sdk_generate_service_manifest(
    SERVICE_NAME realsense_driver
    MANIFEST flowstate/realsense_driver.manifest.textproto
    DEFAULT_CONFIGURATION flowstate/realsense_driver_default_config.textproto
    PARAMETER_DESCRIPTOR flowstate/realsense_driver_config.proto
    PROTOS_TARGET realsense_driver_protos
  )
  install(FILES
          ${CMAKE_CURRENT_BINARY_DIR}/realsense_driver_protos.desc
          ${CMAKE_CURRENT_BINARY_DIR}/service_manifest.binarypb
          ${CMAKE_CURRENT_BINARY_DIR}/default_config.binarypb
          DESTINATION "share/${PROJECT_NAME}"
  )
  add_executable(flowstate_service src/flowstate_service.cc)
  target_link_libraries(flowstate_service
    flowstate_realsense
    intrinsic_sdk_cmake::intrinsic_sdk_cmake
    realsense_driver_protos
    rclcpp::rclcpp
  )
  add_dependencies(flowstate_service
    realsense_driver_manifest
    realsense_driver_default_config
  )
  install(TARGETS flowstate_service
          DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "intrinsic_sdk_ros not found")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(helpers_test test/helpers_test.cc)
  target_link_libraries(helpers_test
    flowstate_realsense_helpers
    absl::status
    absl::statusor
    absl::strings
    ${sensor_msgs_TARGETS}
  )
endif()

ament_package()
