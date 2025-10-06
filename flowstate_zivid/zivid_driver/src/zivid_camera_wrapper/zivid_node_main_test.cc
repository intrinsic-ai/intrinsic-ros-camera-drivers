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
#include "snapshot_interfaces/srv/snapshot.hpp"

#include <fstream>
#include <memory>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "rclcpp/rclcpp.hpp"

#include "flowstate/zivid_driver_config.pb.h"

// intrinsic_proto::config::RuntimeContext
// GetRuntimeContext()
// {
//   intrinsic_proto::config::RuntimeContext runtime_context;
//   std::ifstream runtime_context_file;
//   runtime_context_file.open("/etc/intrinsic/runtime_config.pb", std::ios::binary);
//   if (!runtime_context.ParseFromIstream(&runtime_context_file)) {
//     // Return default context for running locally
//     std::cerr << "Warning: using default RuntimeContext\n";
//   }
//   return runtime_context;
// }

// void StartZenohBridge(const std::string & zenoh_router_address)
// {
//   // Start the zenoh bridge
//   auto zenoh_pkg_location = ament_index_cpp::get_package_prefix("zenoh_bridge_dds");
//   if (!zenoh_pkg_location.empty()) {
//     std::string cmd = zenoh_pkg_location + "/lib/zenoh_bridge_dds/zenoh_bridge_dds" + \
//       " -m client" + \
//       " -e " + zenoh_router_address + \
//       " --no-multicast-scouting &";
//     std::system(cmd.c_str());
//     RCLCPP_INFO(rclcpp::get_logger("zivid_driver_main"), "Started Zenoh bridge");
//   }
// }

void fatal_error(const rclcpp::Logger & logger, const std::string & message)
{
  RCLCPP_ERROR_STREAM(logger, message);
  throw std::runtime_error(message);
}

void set_settings_2d(const std::shared_ptr<rclcpp::Node> & node, const std::string & target_node_name)
{
  RCLCPP_INFO(node->get_logger(), "Setting parameter `settings_2d_yaml` for node %s", target_node_name.c_str());
  const std::string settings_2d_yaml =
    R"(
__version__:
  serializer: 1
  data: 3
Settings2D:
  Acquisitions:
    - Acquisition:
        Aperture: 2.83
        Brightness: 1.0
        ExposureTime: 10000
        Gain: 2.5
)";

  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, target_node_name);
  while (!param_client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the parameters client to appear...");
  }

  auto result =
    param_client->set_parameters({rclcpp::Parameter("settings_2d_yaml", settings_2d_yaml)});
  if (
    rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(30)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to set `settings_2d_yaml` parameter");
  }
}

void set_settings(const std::shared_ptr<rclcpp::Node> & node, const std::string & target_node_name)
{
  RCLCPP_INFO(node->get_logger(), "Setting capture parameters for node %s", target_node_name.c_str());
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, target_node_name);
  while (!param_client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the parameters client to appear...");
  }
  auto parameters = {
    rclcpp::Parameter("aperture", 5.66),
    rclcpp::Parameter("exposure_time", 0.008333),  // 8333 us
    rclcpp::Parameter("outlier_removal_enabled", true),
    rclcpp::Parameter("outlier_removal_threshold", 5.0),
  };
  auto result = param_client->set_parameters(parameters);
  if (
    rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(30)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to set capture parameters");
  }
}

void set_srgb(const std::shared_ptr<rclcpp::Node> & node, const std::string & target_node_name)
{
  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, target_node_name);
  while (!param_client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the parameters client to appear...");
  }

  auto result = param_client->set_parameters({rclcpp::Parameter("color_space", "srgb")});
  if (
    rclcpp::spin_until_future_complete(node, result, std::chrono::seconds(30)) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to set `color_space` parameter");
  }
}

void discover()
{
  auto client_node = rclcpp::Node::make_shared("discovery_client");

  auto discovery_client = client_node->create_client<snapshot_interfaces::srv::Discover>("/cameras/discover"); // ("/zivid_discovery/discover");

  if (!discovery_client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(client_node->get_logger(), "Discovery service not available");
    return;
  }
  auto request = std::make_shared<snapshot_interfaces::srv::Discover::Request>();
  
  auto future = discovery_client->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(client_node, future, std::chrono::seconds(50)) == 
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();

  if (response->success && !response->cameras.empty()) {
      RCLCPP_INFO(client_node->get_logger(), "Discovered %zu cameras:", response->cameras.size());
      for (size_t i = 0; i < response->cameras.size(); ++i) {
        const auto& camera = response->cameras[i];

        RCLCPP_INFO(client_node->get_logger(), "Camera serial: %s", camera.camera_id.c_str());
      }
    } else {
      RCLCPP_WARN(client_node->get_logger(), "No cameras discovered or discovery failed");
    }
  } else {
    fatal_error(client_node->get_logger(), "Discovery service call timed out");
  }
}

auto create_snapshot_client(std::shared_ptr<rclcpp::Node> & node, std::string service_name)
{
  auto client = node->create_client<snapshot_interfaces::srv::Snapshot>(service_name);
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the snapshot service %s to appear...", service_name.c_str());
  }

  RCLCPP_INFO(node->get_logger(), "%s service is available", service_name.c_str());
  return client;
}

void call_capture_color_image_service_once(std::shared_ptr<rclcpp::Node> & client_node, const std::string & target_node_name, const std::string& service_name) {

  // Create service client
  auto client = create_snapshot_client(client_node, target_node_name + service_name);

  // Wait for service to be available
  if (!client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(client_node->get_logger(), "Service %s not available", service_name.c_str());
    return;
  }

  // Create subscription BEFORE calling capture so it's ready to receive data
  auto color_image_color_subscription = client_node->create_subscription<sensor_msgs::msg::Image>(
    target_node_name + "/color/image_color", 2, [&](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(client_node->get_logger(), "Received image of size %d x %d", msg->width, msg->height);
    });
  
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Create request and call service
  auto request = std::make_shared<snapshot_interfaces::srv::Snapshot::Request>();
  RCLCPP_INFO(client_node->get_logger(), "Calling capture service %s...", service_name.c_str());
  
  auto future = client->async_send_request(request);

  // Wait for response with timeout
  if (rclcpp::spin_until_future_complete(client_node, future, std::chrono::seconds(30)) == 
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(client_node->get_logger(), "Snapshot successful.");
      RCLCPP_INFO(client_node->get_logger(), "  - Received %zu point clouds", response->point_clouds.size());
      RCLCPP_INFO(client_node->get_logger(), "  - Received %zu images", response->images.size());
    } else {
      RCLCPP_ERROR(client_node->get_logger(), "Snapshot failed: %s", response->error_message.c_str());
    }
  } else {
    RCLCPP_ERROR(client_node->get_logger(), "Snapshot service call timed out");
  }
}

void call_capture_color_image_service(std::shared_ptr<rclcpp::Node> & client_node, const std::string & target_node_name, const std::string& service_name) {

  // Create service client
  auto client = create_snapshot_client(client_node, target_node_name + service_name);

  auto trigger_capture = [&]() {
    RCLCPP_INFO(client_node->get_logger(), "Triggering 2d capture");
    client->async_send_request(std::make_shared<snapshot_interfaces::srv::Snapshot::Request>());
  };


  auto color_image_color_subscription = client_node->create_subscription<sensor_msgs::msg::Image>(
    target_node_name + "/color/image_color", 2, [&](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(client_node->get_logger(), "Received image of size %d x %d", msg->width, msg->height);
      trigger_capture();
    });
  
  trigger_capture();

  rclcpp::spin(client_node);
  
}

void call_depth_image_service(std::shared_ptr<rclcpp::Node> & client_node, const std::string & target_node_name, const std::string& service_name) {

  // Create service client
  auto client = create_snapshot_client(client_node, target_node_name + service_name);

  auto trigger_capture = [&]() {
    RCLCPP_INFO(client_node->get_logger(), "Triggering depth capture");
    client->async_send_request(std::make_shared<snapshot_interfaces::srv::Snapshot::Request>());
  };

  auto depth_image_subscription = client_node->create_subscription<sensor_msgs::msg::Image>(
    target_node_name + "/depth/image", 2, [&](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(client_node->get_logger(), "Received depth image of size %d x %d", msg->width, msg->height);
      trigger_capture();
    });
  
  trigger_capture();

  rclcpp::spin(client_node);
  
}

void call_depth_image_service_once(std::shared_ptr<rclcpp::Node> & client_node, const std::string & target_node_name, const std::string& service_name) {

  // Create service client
  auto client = create_snapshot_client(client_node, target_node_name + service_name);

  // Wait for service to be available
  if (!client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(client_node->get_logger(), "Service %s not available", service_name.c_str());
    return;
  }

  // Create subscription BEFORE calling capture so it's ready to receive data
  auto points_xyzrgba_subscription = client_node->create_subscription<sensor_msgs::msg::Image>(
    target_node_name + "/depth/image", 10, [&](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(
        client_node->get_logger(), "Received depth image of size %d x %d", msg->width, msg->height);
    });

  // Give subscription time to be established
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Create request and call service
  // Create request and call service
  auto request = std::make_shared<snapshot_interfaces::srv::Snapshot::Request>();
  RCLCPP_INFO(client_node->get_logger(), "Calling capture service %s...", service_name.c_str());
  
  auto future = client->async_send_request(request);

  // Wait for response with timeout
  if (rclcpp::spin_until_future_complete(client_node, future, std::chrono::seconds(30)) == 
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(client_node->get_logger(), "Snapshot successful.");
      RCLCPP_INFO(client_node->get_logger(), "  - Received %zu point clouds", response->point_clouds.size());
      RCLCPP_INFO(client_node->get_logger(), "  - Received %zu images", response->images.size());
    } else {
      RCLCPP_ERROR(client_node->get_logger(), "Snapshot failed: %s", response->error_message.c_str());
    }
  } else {
    RCLCPP_ERROR(client_node->get_logger(), "Snapshot service call timed out");
  }
}

void call_pc_service_continuous(std::shared_ptr<rclcpp::Node> & client_node, const std::string & target_node_name, const std::string& service_name) {

  // Create service client
  auto client = create_snapshot_client(client_node, target_node_name + service_name);

  auto trigger_capture = [&]() {
    RCLCPP_INFO(client_node->get_logger(), "Triggering capture");
    client->async_send_request(std::make_shared<snapshot_interfaces::srv::Snapshot::Request>());
  };

  auto points_xyzrgba_subscription = client_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    target_node_name + "/points/xyz", 10, [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(
        client_node->get_logger(), "Received point cloud of size %d x %d", msg->width, msg->height);
      trigger_capture();
    });
  
  trigger_capture();

  rclcpp::spin(client_node);
  
}

void call_pc_service_once(std::shared_ptr<rclcpp::Node> & client_node, const std::string & target_node_name, const std::string& service_name) {

  // Create service client
  auto client = create_snapshot_client(client_node, target_node_name + service_name);

  // Wait for service to be available
  if (!client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(client_node->get_logger(), "Service %s not available", service_name.c_str());
    return;
  }

  // Create subscription BEFORE calling capture so it's ready to receive data
  auto points_xyzrgba_subscription = client_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    target_node_name + "/points/xyz", 10, [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(
        client_node->get_logger(), "Received point cloud of size %d x %d", msg->width, msg->height);
    });

  // Give subscription time to be established
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Create request and call service
  auto request = std::make_shared<snapshot_interfaces::srv::Snapshot::Request>();
  RCLCPP_INFO(client_node->get_logger(), "Calling snapshot service %s...", service_name.c_str());

  auto future = client->async_send_request(request);

  // Wait for response with timeout
  if (rclcpp::spin_until_future_complete(client_node, future, std::chrono::seconds(30)) == 
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(client_node->get_logger(), "Snapshot successful.");
      RCLCPP_INFO(client_node->get_logger(), "  - Received %zu point clouds", response->point_clouds.size());
      RCLCPP_INFO(client_node->get_logger(), "  - Received %zu images", response->images.size());
    } else {
      RCLCPP_ERROR(client_node->get_logger(), "Snapshot failed: %s", response->error_message.c_str());
    }
  } else {
    RCLCPP_ERROR(client_node->get_logger(), "Snapshot service call timed out");
  }
}

void call_capture_normals_service(std::shared_ptr<rclcpp::Node> & client_node, const std::string & target_node_name, const std::string& service_name) {

  set_settings(client_node, target_node_name);
  set_srgb(client_node, target_node_name);

  // Create service client
  auto client = create_snapshot_client(client_node, target_node_name + service_name);

  auto trigger_capture = [&]() {
    RCLCPP_INFO(client_node->get_logger(), "Triggering capture");
    client->async_send_request(std::make_shared<snapshot_interfaces::srv::Snapshot::Request>());
  };

  auto points_normals_subscription = client_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    target_node_name + "/normals/xyz", 10, [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(
        client_node->get_logger(), "Received normals xyz of size %d x %d", msg->width, msg->height);
      trigger_capture();
    });
  
  trigger_capture();

  rclcpp::spin(client_node);
  
}

void call_capture_normals_service_once(std::shared_ptr<rclcpp::Node> & client_node, const std::string & target_node_name, const std::string& service_name) {

  set_settings(client_node, target_node_name);
  set_srgb(client_node, target_node_name);

  // Create service client
  auto client = create_snapshot_client(client_node, target_node_name + service_name);

  // Wait for service to be available
  if (!client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(client_node->get_logger(), "Service %s not available", service_name.c_str());
    return;
  }

  // Create subscription BEFORE calling capture so it's ready to receive data
  auto points_normals_subscription = client_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    target_node_name +"/normals/xyz", 10, [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) -> void {
      RCLCPP_INFO(
        client_node->get_logger(), "Received normals xyz of size %d x %d", msg->width, msg->height);
  });

  // Give subscription time to be established
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Create request and call service
  auto request = std::make_shared<snapshot_interfaces::srv::Snapshot::Request>();
  RCLCPP_INFO(client_node->get_logger(), "Calling snapshot service %s...", service_name.c_str());
  
  auto future = client->async_send_request(request);

  // Wait for response with timeout
  if (rclcpp::spin_until_future_complete(client_node, future, std::chrono::seconds(30)) == 
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(client_node->get_logger(), "Snapshot successful.");
      RCLCPP_INFO(client_node->get_logger(), "  - Received %zu point clouds", response->point_clouds.size());
      RCLCPP_INFO(client_node->get_logger(), "  - Received %zu images", response->images.size());
    } else {
      RCLCPP_ERROR(client_node->get_logger(), "Snapshot failed: %s", response->error_message.c_str());
    }
  } else {
    RCLCPP_ERROR(client_node->get_logger(), "Snapshot service call timed out");
  }
}

void describe(const std::string & target_node_name, const std::string& service_name)
{
  auto client_node = rclcpp::Node::make_shared("describe_client");

  auto describe_client = client_node->create_client<snapshot_interfaces::srv::Describe>(target_node_name + service_name);

  if (!describe_client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(client_node->get_logger(), "Describe service not available");
    return;
  }
  auto request = std::make_shared<snapshot_interfaces::srv::Describe::Request>();

  auto future = describe_client->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(client_node, future, std::chrono::seconds(50)) == 
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();

    if (response->success && !response->sensors.empty()) {
      RCLCPP_INFO(client_node->get_logger(), "Got description for %s:", target_node_name.c_str());
      for (const auto& sensor : response->sensors) {
        RCLCPP_INFO(client_node->get_logger(), "Sensor name: %s", sensor.sensor_name.c_str());
        RCLCPP_INFO(client_node->get_logger(), "  topic name: %s", sensor.topic_name.c_str());
        RCLCPP_INFO(client_node->get_logger(), "  Sensor type: %u", sensor.sensor_type);
        
        const auto& transform = sensor.camera_t_sensor.transform;
        RCLCPP_INFO(client_node->get_logger(), "  camera_t_sensor: translation(x,y,z): (%f, %f, %f), rotation(x,y,z,w): (%f, %f, %f, %f)",
            transform.translation.x, transform.translation.y, transform.translation.z,
            transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
        if (!sensor.info.empty()) {
            const auto& cam_info = sensor.info[0];
            RCLCPP_INFO(client_node->get_logger(), "  CameraInfo: width=%d, height=%d, distortion_model=%s",
                cam_info.width, cam_info.height, cam_info.distortion_model.c_str());
        } else {
            RCLCPP_INFO(client_node->get_logger(), "  CameraInfo: None");
        }
        RCLCPP_INFO(client_node->get_logger(),"----------------------");
      }
    } else {
      RCLCPP_WARN(client_node->get_logger(), "Describe service failed");
    }
  } else {
    fatal_error(client_node->get_logger(), "Describe service call timed out");
  }
}

// void connect_camera(std::shared_ptr<rclcpp::Node> & client_node, const std::string & target_node_name)
// {
//   RCLCPP_INFO(client_node->get_logger(), "Attempting to connect to camera node: %s", target_node_name.c_str());
//   auto connect_client = client_node->create_client<std_srvs::srv::Trigger>(target_node_name + "/connect");
//   while (!connect_client->wait_for_service(std::chrono::seconds(3))) {
//     if (!rclcpp::ok()) {
//       fatal_error(client_node->get_logger(), "Client interrupted while waiting for service to appear.");
//     }
//     RCLCPP_INFO(client_node->get_logger(), "Waiting for the connect service to appear...");
//   }

//   auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
//   auto future = connect_client->async_send_request(request);

//   if (rclcpp::spin_until_future_complete(client_node, future, std::chrono::seconds(40)) != rclcpp::FutureReturnCode::SUCCESS) {
//     fatal_error(client_node->get_logger(), "Connect service call failed or timed out.");
//   }
//   auto response = future.get();
//   if (!response->success) {
//     fatal_error(client_node->get_logger(), "Failed to connect to camera: " + response->message);
//   }
//   RCLCPP_INFO(client_node->get_logger(), "Successfully connected to camera: %s", response->message.c_str());
// }

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


int main(int argc, char** argv) 
{

  // In a Kubernetes environment, it's crucial to explicitly configure Zenoh.
  // We'll use an environment variable to specify the router's endpoint.
  const char* zenoh_router_env = std::getenv("ZENOH_ROUTER_ENDPOINT");
  if (zenoh_router_env) {
    std::string zenoh_config_override = "connect/endpoints=[\"";
    zenoh_config_override += zenoh_router_env;
    zenoh_config_override += "\"];scouting/multicast/enabled=false";
    
    printf("ZENOH_ROUTER_ENDPOINT is set. Overriding Zenoh config: %s\n", zenoh_config_override.c_str());
    setenv("ZENOH_CONFIG_OVERRIDE", zenoh_config_override.c_str(), 1);
  } else {
    printf("ZENOH_ROUTER_ENDPOINT is not set. Using default Zenoh configuration.\n");
  }


  rclcpp::init(argc, argv);

  try {
      std::string camera_list = exec("/usr/bin/ZividListCameras");
      RCLCPP_INFO(rclcpp::get_logger("zivid_driver_main"), "ZividListCameras output:\n%s", camera_list.c_str());
  } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("zivid_driver_main"), "Failed to execute ZividListCameras: %s", e.what());
  }

  auto spawner_node = zivid_camera_wrapper::ZividCameraWrapper::Create();
  if (!spawner_node.ok()) {
    RCLCPP_ERROR_STREAM((*spawner_node)->get_logger(),
                        "Zivid spawner failed to start: " << spawner_node.status());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }


  std::thread spin_thread([&spawner_node]() {
    rclcpp::spin(*spawner_node);
  });

  // try {
  //     std::string camera_list = exec("ros2 service list");
  //     RCLCPP_INFO(rclcpp::get_logger("zivid_driver_main"), "ros2 service list output:\n%s", camera_list.c_str());
  // } catch (const std::exception& e) {
  //     RCLCPP_ERROR(rclcpp::get_logger("zivid_driver_main"), "Failed to execute ros2 service list: %s", e.what());
  // }

  // Give camera nodes time to start up
  std::this_thread::sleep_for(std::chrono::seconds(3));


  // call discovery service
  discover();

  // Get the generated camera node names
  auto camera_node_names = (*spawner_node)->getCameraNodeNames();
  RCLCPP_INFO((*spawner_node)->get_logger(), "Generated camera node names:");
  for (const auto& node_name : camera_node_names) {
    RCLCPP_INFO((*spawner_node)->get_logger(), "  - %s", node_name.c_str());
  }

  // Use the first camera for captures if available
  std::string target_camera_node = "";
  if (!camera_node_names.empty()) {
    target_camera_node = "/" + camera_node_names[0];
    RCLCPP_INFO((*spawner_node)->get_logger(), "Using camera node: %s", target_camera_node.c_str());
  } else {
    RCLCPP_ERROR((*spawner_node)->get_logger(), "No camera nodes available!");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  auto client_node = rclcpp::Node::make_shared("snapshot_client");

  // Connect to the camera before attempting to capture
  // connect_camera(client_node, target_camera_node);

  // RCLCPP_INFO(client_node->get_logger(), "Starting continuous snapshot mode. Press Ctrl+C to exit");

  // capture 2d image service - continuous snapshot mode
  // call_capture_color_image_service(client_node, target_camera_node, "/snapshot");

  // capture 2d image service - single snapshot mode
  call_capture_color_image_service_once(client_node, target_camera_node, "/snapshot");

  // capture depth image service - continuous snapshot mode
  // call_capture_depth_image_service(client_node, target_camera_node, "/snapshot");

  // capture depth image service - single snapshot mode
  // call_capture_depth_image_service_once(client_node, target_camera_node, "/snapshot");

  // capture point cloud service - continuous snapshot mode
  // call_capture_pc_service(client_node, target_camera_node, "/snapshot");

  // capture point cloud service - single snapshot mode
  // call_capture_pc_service_once(client_node, target_camera_node, "/snapshot");

  // capture normals service - continuous snapshot mode
  // call_capture_normals_service(client_node, target_camera_node, "/snapshot");

  // capture normals service - single snapshot mode
  // call_capture_normals_service_once(client_node, target_camera_node, "/snapshot");

  // describe service
  describe(target_camera_node, "/describe");

  // Spin the client_node so it can receive subscription callbacks
  
  rclcpp::spin(client_node);
  
  // The spin_thread will be automatically terminated when main thread exits
  RCLCPP_INFO((*spawner_node)->get_logger(), "Shutting down");

  
  // Wait for the background thread to finish
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  
  // The selected call_... function will spin the client_node.
  RCLCPP_INFO(client_node->get_logger(), "Shutting down");
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}