cmake_minimum_required(VERSION 3.8)


intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME zivid_driver
  MANIFEST flowstate/zivid_driver.manifest.textproto
  DEFAULT_CONFIGURATION flowstate/zivid_driver_default_config.pbtxt
  PARAMETER_DESCRIPTOR flowstate/zivid_driver_config.proto
  PROTOS_TARGET zivid_driver_protos
)

add_library(zivid_camera_node SHARED zivid_camera_node.cc)
target_include_directories(zivid_camera_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(zivid_camera_node
    rclcpp
    std_msgs
    std_srvs
    zivid_camera
    snapshot_interfaces
    sensor_msgs
    rcl_interfaces
    geometry_msgs
)
target_link_libraries(zivid_camera_node Zivid::Core)
install(TARGETS zivid_camera_node DESTINATION lib)


add_library(zivid_nodes
    zivid_camera_wrapper.cc
  )


target_include_directories(zivid_nodes PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(zivid_nodes 
  Zivid::Core
  absl::status
  absl::statusor
  absl::strings
  absl::synchronization
  zivid_camera_node
  )

install(TARGETS zivid_nodes DESTINATION lib)


# ROS dependencies
ament_target_dependencies(zivid_nodes
    rclcpp
    std_msgs
    std_srvs
    zivid_camera
    snapshot_interfaces
    sensor_msgs
    rcl_interfaces
    geometry_msgs
    ament_index_cpp
)



# Add executable
add_executable(zivid_driver_main 
  zivid_node_main.cc)

# Include directories - point to the package root's include directory
target_include_directories(zivid_driver_main PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<INSTALL_INTERFACE:include>
)

find_package(absl REQUIRED)


# Link libraries
target_link_libraries(zivid_driver_main 
    Zivid::Core
    absl::status
    absl::statusor
    absl::strings
    absl::synchronization
    zivid_driver_protos
    zivid_nodes
    # intrinsic_sdk::intrinsic_sdk
    )


add_dependencies(zivid_driver_main 
    zivid_driver_manifest 
    zivid_driver_default_config
  )

# ROS dependencies
ament_target_dependencies(zivid_driver_main
    rclcpp
    std_msgs
    std_srvs
    zivid_camera
    snapshot_interfaces
    sensor_msgs
    rcl_interfaces
    geometry_msgs
)

# Set C++ standard
target_compile_features(zivid_driver_main PUBLIC cxx_std_17)

# Install executable
install(TARGETS zivid_driver_main
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Add executable
  add_executable(zivid_driver_main_test
    zivid_node_main_test.cc
  )

  # Include directories - point to the package root's include directory
  target_include_directories(zivid_driver_main_test PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
      $<INSTALL_INTERFACE:include>
  )

  # Link libraries
  target_link_libraries(zivid_driver_main_test
      Zivid::Core
      absl::status
      absl::statusor
      absl::strings
      absl::synchronization
      zivid_driver_protos
      zivid_nodes
      )


  # ROS dependencies
  ament_target_dependencies(zivid_driver_main_test
      rclcpp
      std_msgs
      std_srvs
      zivid_camera
      snapshot_interfaces
      rcl_interfaces
      geometry_msgs
      ament_index_cpp
  )

  # Set C++ standard
  target_compile_features(zivid_driver_main_test PUBLIC cxx_std_17)

  # Install executable
  install(TARGETS zivid_driver_main_test
      DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install the proto descriptor database
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/zivid_driver_protos.desc
        DESTINATION "share/${PROJECT_NAME}/zivid_driver"
)

# Install the service manifest
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/service_manifest.binarypb
        DESTINATION "share/${PROJECT_NAME}/zivid_driver"
)
# Install the default configuration
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/default_config.binarypb
        DESTINATION "share/${PROJECT_NAME}/zivid_driver"
)
