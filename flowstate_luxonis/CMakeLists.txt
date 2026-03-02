cmake_minimum_required(VERSION 3.8)
project(flowstate_luxonis)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-g -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)

find_package(abseil_cpp_vendor REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(depthai REQUIRED VERBOSE)
find_package(depthai_ros_driver REQUIRED VERBOSE)
find_package(flowstate_camera_common REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(snapshot_interfaces REQUIRED)

find_package(intrinsic_sdk_ros QUIET)

include_directories(include)

add_library(flowstate_luxonis
  src/adapter_node.cc
  src/spawner_node.cc
)
target_link_libraries(flowstate_luxonis
  ${flowstate_camera_common_LIBRARIES}
  absl::status
  absl::synchronization
  rclcpp::rclcpp
  depthai::core
  ${sensor_msgs_TARGETS}
  ${snapshot_interfaces_TARGETS}
)
ament_target_dependencies(flowstate_luxonis
  depthai
  depthai_ros_driver
  flowstate_camera_common
  OpenCV
)
install(TARGETS flowstate_luxonis
        DESTINATION lib)

add_executable(spawner_node src/ros_main.cc)
target_link_libraries(spawner_node flowstate_luxonis)

install(TARGETS spawner_node
        DESTINATION lib/${PROJECT_NAME}
)

if (intrinsic_sdk_ros_FOUND)
  message("intrinsic_sdk_ros found")
  intrinsic_sdk_generate_service_manifest(
    SERVICE_NAME luxonis_driver
    MANIFEST flowstate/luxonis_driver.manifest.textproto
    DEFAULT_CONFIGURATION flowstate/luxonis_driver_default_config.textproto
    PARAMETER_DESCRIPTOR flowstate/luxonis_driver_config.proto
    PROTOS_TARGET luxonis_driver_protos
  )
  install(FILES
          ${CMAKE_CURRENT_BINARY_DIR}/luxonis_driver_protos.desc
          ${CMAKE_CURRENT_BINARY_DIR}/service_manifest.binarypb
          ${CMAKE_CURRENT_BINARY_DIR}/default_config.binarypb
          DESTINATION "share/${PROJECT_NAME}"
  )
  add_executable(flowstate_service src/flowstate_service.cc)
  target_link_libraries(flowstate_service
    flowstate_luxonis
    intrinsic_sdk_cmake::intrinsic_sdk_cmake
    luxonis_driver_protos
    rclcpp::rclcpp
  )
  add_dependencies(flowstate_service
    luxonis_driver_manifest
    luxonis_driver_default_config
  )
  install(TARGETS flowstate_service
          DESTINATION lib/${PROJECT_NAME})
else()
  message("intrinsic_sdk_ros not found")
endif()

ament_package()
