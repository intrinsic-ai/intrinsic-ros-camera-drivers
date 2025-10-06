# Root CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(zivid_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
# Find Zivid SDK first (required by zivid_camera)
find_package(Zivid 2.15.0 COMPONENTS Core REQUIRED)
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
# find_package(rosidl_default_generators REQUIRED)
# find_package(intrinsic_sdk REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#     msg/DiscoveredIpsCamera.msg
#     srv/DiscoverIpsCameras.srv
#   )

# add_subdirectory(src/listener_service)
add_subdirectory(src/zivid_camera_wrapper)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# add_dependencies(zivid_wrapper 
#     zivid_driver_manifest 
#     zivid_driver_default_config
#   )
# Install launch directory
# install(
#   DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# Export include directories
ament_export_include_directories(include)

ament_package()
