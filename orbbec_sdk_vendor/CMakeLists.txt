cmake_minimum_required(VERSION 3.10)
project(orbbec_sdk_vendor CXX)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_vendor(orbbec_sdk_vendor
  VCS_URL https://github.com/orbbec/OrbbecSDK_v2
  VCS_VERSION v2.4.11
)

ExternalProject_Get_Property(orbbec_sdk_vendor BINARY_DIR)

install(FILES
        "${BINARY_DIR}/linux_x86_64/lib/libspdlog.so.1.14.1"
        "${BINARY_DIR}/linux_x86_64/lib/libspdlog.so.1.14"
        DESTINATION "opt/${PROJECT_NAME}/lib/")

ament_package()
