cmake_minimum_required(VERSION 3.8)
project(flowstate_orbbec)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-g -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(orbbec_camera REQUIRED)
find_package(rclcpp REQUIRED)
find_package(snapshot_interfaces REQUIRED)

set(OrbbecSDK_DIR ${CMAKE_INSTALL_PREFIX}/../OrbbecSDK/lib)
find_package(OrbbecSDK REQUIRED)

find_package(intrinsic_sdk_ros QUIET)

include_directories(include)

add_library(flowstate_orbbec src/spawner_node.cpp)
target_compile_definitions(flowstate_orbbec PUBLIC ROS_JAZZY)
target_link_libraries(flowstate_orbbec
  rclcpp::rclcpp
  OrbbecSDK
  ${snapshot_interfaces_TARGETS}
)
ament_target_dependencies(flowstate_orbbec
  cv_bridge
  orbbec_camera
)

add_executable(spawner_node src/ros_main.cpp)
target_link_libraries(spawner_node flowstate_orbbec)

install(TARGETS spawner_node
        DESTINATION lib/${PROJECT_NAME}
)

if (intrinsic_sdk_ros_FOUND)
  message("intrinsic_sdk_ros found")
  intrinsic_sdk_generate_service_manifest(
    SERVICE_NAME orbbec_driver
    MANIFEST flowstate/orbbec_driver.manifest.textproto
    DEFAULT_CONFIGURATION flowstate/orbbec_driver_default_config.textproto
    PARAMETER_DESCRIPTOR flowstate/orbbec_driver_config.proto
    PROTOS_TARGET orbbec_driver_protos
  )
  install(FILES
          ${CMAKE_CURRENT_BINARY_DIR}/orbbec_driver_protos.desc
          ${CMAKE_CURRENT_BINARY_DIR}/service_manifest.binarypb
          ${CMAKE_CURRENT_BINARY_DIR}/default_config.binarypb
          DESTINATION "share/${PROJECT_NAME}"
  )
  add_executable(flowstate_service src/flowstate_service.cc)
  target_link_libraries(flowstate_service
    flowstate_orbbec
    intrinsic_sdk_cmake::intrinsic_sdk_cmake
    orbbec_driver_protos
    rclcpp::rclcpp
  )
  add_dependencies(flowstate_service
    orbbec_driver_manifest
    orbbec_driver_default_config
  )
  install(TARGETS flowstate_service
          DESTINATION lib/${PROJECT_NAME})
else()
  message("intrinsic_sdk_ros not found")
endif()

ament_package()
