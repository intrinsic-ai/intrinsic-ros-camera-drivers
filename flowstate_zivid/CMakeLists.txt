cmake_minimum_required(VERSION 3.8)
project(flowstate_zivid)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
# Find Zivid SDK first (required by zivid_camera)
find_package(Zivid 2.17.0 COMPONENTS Core REQUIRED)
message(STATUS "Found Zivid SDK version ${Zivid_VERSION}")

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(zivid_camera REQUIRED)
find_package(snapshot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(intrinsic_sdk_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(abseil_cpp_vendor REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)


intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME zivid_driver
  MANIFEST flowstate/zivid_driver.manifest.textproto
)

add_library(zivid_camera_node SHARED src/adapter_node.cc)

target_include_directories(
  zivid_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(zivid_camera_node rclcpp
    std_msgs
    std_srvs
    zivid_camera
    snapshot_interfaces
    sensor_msgs
    rcl_interfaces
    geometry_msgs
    image_transport
)
target_link_libraries(
  zivid_camera_node
  Zivid::Core
  absl::base
  absl::core_headers
  absl::status
  absl::statusor
  absl::strings
  absl::synchronization
  yaml-cpp
)
install(TARGETS zivid_camera_node DESTINATION lib)


add_library(zivid_nodes src/spawner_node.cc)


target_include_directories(
  zivid_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  zivid_nodes
  Zivid::Core
  absl::base
  absl::core_headers
  absl::status
  absl::statusor
  absl::strings
  absl::synchronization
  zivid_camera_node
)

install(TARGETS zivid_nodes DESTINATION lib)


# ROS dependencies
ament_target_dependencies(zivid_nodes rclcpp
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
  src/flowstate_service.cc)

# Include directories - point to the package root's include directory
target_include_directories(zivid_driver_main PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link libraries
target_link_libraries(zivid_driver_main 
    Zivid::Core
    absl::status
    absl::statusor
    absl::strings
    absl::synchronization
    zivid_nodes
    )


add_dependencies(zivid_driver_main
    zivid_driver_manifest
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
    ament_index_cpp
)

# Set C++ standard
target_compile_features(zivid_driver_main PUBLIC cxx_std_17)

# Install executable
install(TARGETS zivid_driver_main
    DESTINATION lib/${PROJECT_NAME}
)

# Export include directories
ament_export_include_directories(include)

# Install the service manifest
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/service_manifest.binarypb
        DESTINATION "share/${PROJECT_NAME}/zivid_driver"
)

ament_package()
