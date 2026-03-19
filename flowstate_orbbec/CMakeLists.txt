cmake_minimum_required(VERSION 3.8)
project(flowstate_orbbec)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-g -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)

find_package(abseil_cpp_vendor REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(flowstate_camera_common REQUIRED)
find_package(orbbec_camera REQUIRED)
find_package(orbbec_camera_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(OpenCV 4 REQUIRED
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)
find_package(OrbbecSDK REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(snapshot_interfaces REQUIRED)

find_package(intrinsic_sdk_ros QUIET)

include_directories(include)

add_library(flowstate_orbbec
  src/adapter_node.cc
  src/spawner_node.cc
)
target_compile_definitions(flowstate_orbbec PUBLIC ROS_JAZZY)
target_link_libraries(flowstate_orbbec
  ${flowstate_camera_common_LIBRARIES}
  absl::status
  absl::statusor
  absl::synchronization
  rclcpp::rclcpp
  ob::OrbbecSDK
  ${orbbec_camera_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${snapshot_interfaces_TARGETS}
)
ament_target_dependencies(flowstate_orbbec
  flowstate_camera_common
  OpenCV
  orbbec_camera
  rcl_interfaces
)

add_executable(spawner_node src/ros_main.cc)
target_link_libraries(spawner_node flowstate_orbbec)

install(TARGETS spawner_node
        DESTINATION lib/${PROJECT_NAME}
)

if (intrinsic_sdk_ros_FOUND)
  message("intrinsic_sdk_ros found")
  intrinsic_sdk_generate_service_manifest(
    SERVICE_NAME orbbec_gemini_driver
    MANIFEST flowstate/orbbec_gemini_driver.manifest.textproto
    DEFAULT_CONFIGURATION flowstate/orbbec_gemini_driver_default_config.textproto
    PARAMETER_DESCRIPTOR flowstate/orbbec_gemini_driver_config.proto
    PROTOS_TARGET orbbec_gemini_driver_protos
  )
  install(FILES
          ${CMAKE_CURRENT_BINARY_DIR}/orbbec_gemini_driver_protos.desc
          ${CMAKE_CURRENT_BINARY_DIR}/service_manifest.binarypb
          ${CMAKE_CURRENT_BINARY_DIR}/default_config.binarypb
          DESTINATION "share/${PROJECT_NAME}"
  )
  add_executable(flowstate_service src/flowstate_service.cc)
  target_link_libraries(flowstate_service
    flowstate_orbbec
    intrinsic_sdk_cmake::intrinsic_sdk_cmake
    orbbec_gemini_driver_protos
    rclcpp::rclcpp
  )
  add_dependencies(flowstate_service
    orbbec_gemini_driver_manifest
    orbbec_gemini_driver_default_config
  )
  install(TARGETS flowstate_service
          DESTINATION lib/${PROJECT_NAME})
else()
  message("intrinsic_sdk_ros not found")
endif()

ament_package()
