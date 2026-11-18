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
project(flowstate_zed)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-g -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)

find_package(abseil_cpp_vendor REQUIRED)
find_package(absl REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(flowstate_camera_common REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(snapshot_interfaces REQUIRED)
find_package(zed_components REQUIRED)
# The ZED SDK's CMake config uses commands that newer CMake deprecates.
cmake_policy(PUSH)
if(POLICY CMP0146)
  cmake_policy(SET CMP0146 OLD)
endif()
if(POLICY CMP0153)
  cmake_policy(SET CMP0153 OLD)
endif()
find_package(ZED REQUIRED)
cmake_policy(POP)
find_package(CUDAToolkit REQUIRED)

# Hardware- and SDK-free core library, unit-tested without CUDA or ZED SDK.
add_library(flowstate_zed_core STATIC
  src/camera_discovery.cc
  src/image_conversion.cc
  src/zed_parameters.cc
)
target_include_directories(flowstate_zed_core PUBLIC include)
target_link_libraries(flowstate_zed_core PUBLIC
  absl::span
  absl::status
  absl::statusor
  absl::strings
  absl::synchronization
  absl::time
  rclcpp::rclcpp
  ${rcl_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

add_executable(spawner_node
  src/adapter_node.cc
  src/ros_main.cc
  src/spawner_node.cc
)
target_include_directories(spawner_node PRIVATE include)
target_include_directories(spawner_node SYSTEM PRIVATE
  ${ZED_INCLUDE_DIRS}
  ${CUDAToolkit_INCLUDE_DIRS}
)
target_link_directories(spawner_node PRIVATE
  ${ZED_LIBRARY_DIR}
  ${CUDAToolkit_LIBRARY_DIR}
)
target_link_libraries(spawner_node
  flowstate_zed_core
  ${ZED_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${snapshot_interfaces_TARGETS}
)
ament_target_dependencies(spawner_node
  flowstate_camera_common
  zed_components
)
# libsl_zed.so needs libraries that only the nvidia runtime provides
# (libcuda.so.1, the L4T libnv* ones); they are absent during docker build.
target_link_options(spawner_node PRIVATE -Wl,--allow-shlib-undefined)

install(TARGETS spawner_node
        DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(image_conversion_test test/image_conversion_test.cc)
  target_link_libraries(image_conversion_test flowstate_zed_core)

  ament_add_gtest(zed_parameters_test test/zed_parameters_test.cc)
  target_link_libraries(zed_parameters_test flowstate_zed_core)

  ament_add_gtest(camera_discovery_test test/camera_discovery_test.cc)
  target_link_libraries(camera_discovery_test flowstate_zed_core)

  ament_add_gtest(frame_cache_test test/frame_cache_test.cc)
  target_link_libraries(frame_cache_test flowstate_zed_core)
endif()

ament_package()
