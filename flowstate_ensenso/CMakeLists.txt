cmake_minimum_required(VERSION 3.8)
project(flowstate_ensenso)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_compile_definitions(ROS2)

find_package(abseil_cpp_vendor REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(absl REQUIRED)
find_package(flowstate_camera_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(snapshot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ensenso_camera REQUIRED)
find_package(intrinsic_sdk_ros QUIET)

find_path(NXLIB_INCLUDE_DIR nxLib.h PATHS /opt/ensenso/development/c/include)
find_library(NXLIB_LIBRARY NxLib64 PATHS /opt/ensenso/development/c/lib64)

if(NOT NXLIB_INCLUDE_DIR OR NOT NXLIB_LIBRARY)
  message(FATAL_ERROR "NxLib not found! Please ensure Ensenso SDK is installed.")
endif()

add_library(flowstate_ensenso SHARED
  src/adapter_node.cc
  src/spawner_node.cc
)

target_include_directories(flowstate_ensenso PUBLIC
  include
)

target_include_directories(flowstate_ensenso PRIVATE
  ${NXLIB_INCLUDE_DIR}
)

find_library(ENSENSO_CAMERA_LIB 
  NAMES ensenso_camera ensenso_camera_node ensenso_camera_component
  PATHS /opt/ros/underlay/install/ensenso_camera/lib /opt/ros/underlay/install/lib
  REQUIRED
)

target_link_libraries(flowstate_ensenso
  ${flowstate_camera_common_LIBRARIES}
  absl::status
  absl::statusor
  absl::synchronization
  absl::strings
  ${NXLIB_LIBRARY}
  ${ENSENSO_CAMERA_LIB} # <--- Links the absolute path of the .so file
)

ament_target_dependencies(flowstate_ensenso
  flowstate_camera_common
  ensenso_camera
  rclcpp_components
  pcl_conversions
  common_interfaces
  cv_bridge
  diagnostic_msgs
  image_geometry
  image_transport
  pcl_ros
  rclcpp_action
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  geometry_msgs
  rcl_interfaces
  rclcpp
  sensor_msgs
  snapshot_interfaces
)

add_executable(ensenso_spawner_node src/ros_main.cc)
target_link_libraries(ensenso_spawner_node flowstate_ensenso)
ament_target_dependencies(ensenso_spawner_node rclcpp)

if (intrinsic_sdk_ros_FOUND)
  message(STATUS "intrinsic_sdk_ros found")
  intrinsic_sdk_generate_service_manifest(
    SERVICE_NAME ensenso_driver
    MANIFEST flowstate/ensenso_driver.manifest.textproto
    DEFAULT_CONFIGURATION flowstate/ensenso_driver_default_config.textproto
    PARAMETER_DESCRIPTOR flowstate/ensenso_driver_config.proto
    PROTOS_TARGET ensenso_driver_protos
  )

  install(FILES
          ${CMAKE_CURRENT_BINARY_DIR}/ensenso_driver_protos.desc
          ${CMAKE_CURRENT_BINARY_DIR}/service_manifest.binarypb
          ${CMAKE_CURRENT_BINARY_DIR}/default_config.binarypb
          DESTINATION "share/${PROJECT_NAME}"
  )

  add_executable(flowstate_service src/flowstate_service.cc)
  target_link_libraries(flowstate_service
    flowstate_ensenso
    intrinsic_sdk_cmake::intrinsic_sdk_cmake
    ensenso_driver_protos
  )
  ament_target_dependencies(flowstate_service rclcpp)
  add_dependencies(flowstate_service ensenso_driver_manifest ensenso_driver_default_config)
  
  install(TARGETS flowstate_service
          DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "intrinsic_sdk_ros not found")
endif()

install(TARGETS flowstate_ensenso 
  DESTINATION lib
)

install(TARGETS flowstate_ensenso ensenso_spawner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()
