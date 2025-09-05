cmake_minimum_required(VERSION 3.8)
project(flowstate_orbbec)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-g -Wall -Wextra -Wpedantic -Wno-missing-field-initializers)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)

# message("orbbec ${CMAKE_INSTALL_PREFIX}")
set(OrbbecSDK_DIR ${CMAKE_INSTALL_PREFIX}/../OrbbecSDK/lib)
find_package(OrbbecSDK REQUIRED)
find_package(orbbec_camera REQUIRED)
find_package(rclcpp REQUIRED)
find_package(snapshot_interfaces REQUIRED)

find_package(intrinsic_sdk_ros QUIET)

# add_library(flowstate_orbbec src/flowstate_orbbec.cpp)
add_executable(flowstate_orbbec_node src/flowstate_orbbec_node.cpp)

target_link_libraries(flowstate_orbbec_node
  rclcpp::rclcpp
  OrbbecSDK
  ${snapshot_interfaces_TARGETS}
)
target_compile_definitions(flowstate_orbbec_node PUBLIC ROS_JAZZY)
ament_target_dependencies(flowstate_orbbec_node
  cv_bridge
  orbbec_camera
)

install(TARGETS flowstate_orbbec_node
  DESTINATION lib/${project_name}
)

ament_package()
