#include <string>
#include <thread>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "zivid_camera_wrapper/zivid_camera_wrapper.h"
#include "snapshot_interfaces/srv/discover.hpp"

#include <fstream>
#include <memory>
#include <cstdio>
#include <array>

#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "rclcpp/rclcpp.hpp"

#include "flowstate/zivid_driver_config.pb.h"

// Helper function to execute a command and capture its standard output.
std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}


int main(int argc, char* argv[])
{

  // Execute ZividListCameras and log the output for debugging purposes.
  try {
      std::string camera_list = exec("ZividListCameras");
      printf("ZividListCameras output:\n%s", camera_list.c_str());
  } catch (const std::exception& e) {
      printf("Failed to execute ZividListCameras: %s", e.what());
  }

  intrinsic_proto::config::RuntimeContext runtime_context;
  std::ifstream runtime_context_file;
  runtime_context_file.open("/etc/intrinsic/runtime_config.pb",
                            std::ios::binary);
  if (!runtime_context.ParseFromIstream(&runtime_context_file)) {
    std::cerr << "Unable to parse runtime context file" << std::endl;
    return EXIT_FAILURE;
  }
  zivid_driver::ZividDriverConfig config;
  if (!runtime_context.config().UnpackTo(&config)) {
    std::cerr << "Unable to parse config proto" << std::endl;
    return EXIT_FAILURE;
  }
  
  rclcpp::init(argc, argv);

  auto spawner_node = zivid_camera_wrapper::ZividCameraWrapper::Create();
  if (!spawner_node.ok()) {
    RCLCPP_ERROR(rclcpp::get_logger("zivid_driver_main"),
                 "Zivid spawner failed to start: %s", spawner_node.status().ToString().c_str());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  std::thread spin_thread([&spawner_node]() {
    RCLCPP_INFO((*spawner_node)->get_logger(),
                "Spinning spawner... Ctrl+C to exit");
    rclcpp::spin(*spawner_node);
  });

  // try {
  //     std::string camera_list = exec("ros2 service list");
  //     RCLCPP_INFO(rclcpp::get_logger("zivid_driver_main"), "ros2 service list output:\n%s", camera_list.c_str());
  // } catch (const std::exception& e) {
  //     RCLCPP_ERROR(rclcpp::get_logger("zivid_driver_main"), "Failed to execute ros2 service list: %s", e.what());
  // }
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  RCLCPP_INFO((*spawner_node)->get_logger(), "Shutting down");
  rclcpp::shutdown();
  spawner_node->reset();
  return EXIT_SUCCESS;
}