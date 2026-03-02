#include "flowstate_zivid/adapter_node.h"

#include <Zivid/Settings.h>

#include <memory>
#include <string>

#include "absl/algorithm/container.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "snapshot_interfaces/msg/image_snapshot.hpp"
#include "yaml-cpp/yaml.h"
#include "zivid_camera/zivid_camera.hpp"

namespace flowstate_zivid {

using snapshot_interfaces::srv::Describe;
using snapshot_interfaces::srv::Snapshot;

AdapterNode::AdapterNode(const std::string& serial,
                         const rclcpp::NodeOptions& options,
                         std::shared_ptr<Zivid::Application> zivid_app)
    : flowstate_common::BaseAdapterNode(serial, {} , "zivid"),
      capture_params_(CaptureParameters::boot_defaults()) {
  InitializeParameters();
  const std::string zivid_node_name = std::string("camera_") + serial;
  const std::string zivid_ns = "zivid/" + zivid_node_name;

  rclcpp::NodeOptions zivid_node_options = options;
  zivid_node_options.append_parameter_override("serial_number", serial)
      .append_parameter_override(
          "settings_yaml", this->get_parameter("settings_yaml").as_string());

  zivid_node_ = std::make_unique<zivid_camera::ZividCamera>(
      zivid_node_name, zivid_ns, zivid_node_options, zivid_app);

  zivid_camera_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, zivid_ns + "/" + zivid_node_name);
  RCLCPP_INFO(get_logger(),
              "Waiting for zivid_camera parameter service to appear...");
  if (!zivid_camera_param_client_->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(),
                 "Timeout waiting for zivid_camera parameter service. This may "
                 "lead to parameter setting failures.");
    throw std::runtime_error("zivid_camera parameter service not available.");
  }

  RCLCPP_INFO(get_logger(),
              "Applying initial default settings to zivid_camera node.");
  const auto initial_settings_yaml = GenerateZividSettings();
  zivid_camera_param_client_->set_parameters(
      {rclcpp::Parameter("settings_yaml", initial_settings_yaml)});

  set_parameters_callback_handle_ =
      this->add_on_set_parameters_callback(std::bind(
          &AdapterNode::SetParametersCallback, this, std::placeholders::_1));

  // Subscribe to color image + camera_info pair
  color_image_sub_ = image_transport::create_camera_subscription(
      this, ColorImageTopic(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& image,
             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
        absl::MutexLock data_lock(&this->data_mutex_);
        data_.color_image = std::move(image);
        data_.camera_info = std::move(camera_info);
      },
      "raw");

  // In the zivid camera node, camera_info is the same for color and depth,
  // so we only need to subscribe to depth image here.
  depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      DepthImageTopic(), 2, [this](sensor_msgs::msg::Image::UniquePtr msg) {
        absl::MutexLock data_lock(&this->data_mutex_);
        data_.depth_image = std::move(msg);
      });

  // Subscribe to normals point cloud
  normal_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      NormalTopic(), 2, [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
        absl::MutexLock data_lock(&this->data_mutex_);
        this->data_.normal_pc = std::move(msg);
      });

  CreateFlowstateServices();
  RCLCPP_INFO(this->get_logger(), "zivid_node_ = %s",
              zivid_node_->get_fully_qualified_name());
  capture_client_ = this->create_client<std_srvs::srv::Trigger>(
      absl::StrFormat("/zivid/camera_%s/capture", serial_.c_str()),
      rclcpp::ServicesQoS(), callback_group_);
  StartExecutorThread();
}

void AdapterNode::InitializeParameters() {
  declare_parameter<double>("exposure_time", capture_params_.exposure_time);
  declare_parameter<double>("ExposureTime",
                            capture_params_.exposure_time);  // Alias
  declare_parameter<double>("gain", capture_params_.gain);
  declare_parameter<double>("Gain", capture_params_.gain);  // Alias
  declare_parameter<double>("gamma", capture_params_.gamma);
  declare_parameter<double>("Gamma", capture_params_.gamma);  // Alias
  declare_parameter<double>("projector_brightness",
                            capture_params_.projector_brightness);
  declare_parameter<double>("brightness", capture_params_.projector_brightness);
  declare_parameter<double>("Brightness",
                            capture_params_.projector_brightness);  // Alias
  declare_parameter<double>("aperture", capture_params_.aperture);
  declare_parameter<double>("Aperture", capture_params_.aperture);  // Alias

  declare_parameter<bool>("outlier_removal_enabled",
                          capture_params_.outlier_removal_enabled);
  declare_parameter<double>("outlier_removal_threshold",
                            capture_params_.outlier_removal_threshold);

  declare_parameter<std::string>("settings_yaml", "");  // For zivid_camera node
  declare_parameter<std::string>("settings_2d_yaml", "");
  declare_parameter<std::string>("settings_2d_file_path", "");
  declare_parameter<std::string>("color_space", "srgb");
  declare_parameter<std::string>("intrinsics_source", "camera");
}

rcl_interfaces::msg::SetParametersResult AdapterNode::SetParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  const bool full_yaml_provided =
      absl::c_find_if(parameters, [](const rclcpp::Parameter& param) {
        return (param.get_name() == "settings_yaml" ||
                param.get_name() == "settings_file_path") &&
               !param.as_string().empty();
      }) != parameters.end();
  bool individual_param_changed = false;

  absl::MutexLock lock(&capture_params_mutex_);
  for (const auto& param : parameters) {
    RCLCPP_INFO_STREAM(get_logger(), "AdapterNode: Setting parameter '"
                                         << param.get_name() << "' ("
                                         << param.get_type_name() << ") to '"
                                         << param.value_to_string() << "'");

    if (param.get_name() == "exposure_time" ||
        param.get_name() == "ExposureTime") {
      RCLCPP_INFO_STREAM(
          get_logger(),
          "Exposure time needs to have microseconds as unit in Zivid API.");
      capture_params_.exposure_time = param.as_double();
      individual_param_changed = true;
    } else if (param.get_name() == "gain" || param.get_name() == "Gain") {
      capture_params_.gain = param.as_double();
      individual_param_changed = true;
    } else if (param.get_name() == "gamma" || param.get_name() == "Gamma") {
      capture_params_.gamma = param.as_double();
      individual_param_changed = true;
    } else if (param.get_name() == "projector_brightness" ||
               param.get_name() == "brightness" ||
               param.get_name() == "Brightness") {
      capture_params_.projector_brightness = param.as_double();
      individual_param_changed = true;
    } else if (param.get_name() == "aperture" ||
               param.get_name() == "Aperture") {
      capture_params_.aperture = param.as_double();
      individual_param_changed = true;
    } else if (param.get_name() == "outlier_removal_enabled" ||
               param.get_name() == "OutlierRemovalEnabled") {
      capture_params_.outlier_removal_enabled = param.as_bool();
      individual_param_changed = true;
    } else if (param.get_name() == "outlier_removal_threshold" ||
               param.get_name() == "OutlierRemovalThreshold") {
      capture_params_.outlier_removal_threshold = param.as_double();
      individual_param_changed = true;
    } else if (param.get_name() == "color_space" ||
               param.get_name() == "intrinsics_source") {
      RCLCPP_INFO_STREAM(get_logger(), "Passing through '"
                                           << param.get_name()
                                           << "' to zivid_camera node.");
      zivid_camera_param_client_->set_parameters({param});
    } else {
      RCLCPP_INFO_STREAM(get_logger(),
                         "Passing through parameter '"
                             << param.get_name() << "' to zivid_camera node: "
                             << zivid_node_->get_fully_qualified_name());
      zivid_camera_param_client_->set_parameters({param});
    }
  }

  if (individual_param_changed && !full_yaml_provided) {
    RCLCPP_INFO(get_logger(),
                "Individual parameter changed. Generating and applying new "
                "settings_yaml.");
    const auto settings_yaml = GenerateZividSettings();
    zivid_camera_param_client_->set_parameters(
        {rclcpp::Parameter("settings_yaml", settings_yaml)});
  }

  return result;
}

std::string AdapterNode::GenerateZividSettings() const {
  RCLCPP_INFO_STREAM(
      get_logger(),
      "Generating Zivid settings from current capture parameters.");
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "__version__";
  out << YAML::Value << YAML::BeginMap << YAML::Key << "serializer"
      << YAML::Value << 1 << YAML::Key << "data" << YAML::Value << 22
      << YAML::EndMap;
  out << YAML::Key << "Settings";
  out << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "Acquisitions";
  out << YAML::Value << YAML::BeginSeq;
  out << YAML::BeginMap;
  out << YAML::Key << "Acquisition";
  out << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "Aperture" << YAML::Value << capture_params_.aperture;
  out << YAML::Key << "Brightness" << YAML::Value
      << capture_params_.projector_brightness;
  out << YAML::Key << "ExposureTime" << YAML::Value
      << capture_params_.exposure_time;
  out << YAML::Key << "Gain" << YAML::Value << capture_params_.gain;
  out << YAML::EndMap;  // Acquisition
  out << YAML::EndMap;
  out << YAML::EndSeq;  // Acquisitions
  out << YAML::Key << "Processing";
  out << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "Color" << YAML::Value << YAML::BeginMap << YAML::Key
      << "Gamma" << YAML::Value << capture_params_.gamma << YAML::EndMap;
  out << YAML::Key << "Filters" << YAML::Value << YAML::BeginMap << YAML::Key
      << "Outlier" << YAML::Value << YAML::BeginMap << YAML::Key << "Removal"
      << YAML::Value << YAML::BeginMap << YAML::Key << "Enabled" << YAML::Value
      << (capture_params_.outlier_removal_enabled ? "yes" : "no") << YAML::Key
      << "Threshold" << YAML::Value << capture_params_.outlier_removal_threshold
      << YAML::EndMap << YAML::EndMap << YAML::EndMap << YAML::EndMap;
  out << YAML::EndMap;  // Settings
  out << YAML::EndMap;
  return out.c_str();
}

std::string AdapterNode::ColorImageTopic() const {
  return absl::StrFormat("/zivid/camera_%s/color/image_color", serial_.c_str());
}

std::string AdapterNode::DepthImageTopic() const {
  return absl::StrFormat("/zivid/camera_%s/depth/image", serial_.c_str());
}

std::string AdapterNode::NormalTopic() const {
  return absl::StrFormat("/zivid/camera_%s/normals/xyz", serial_.c_str());
}

absl::Status AdapterNode::Main() {
  RCLCPP_INFO(get_logger(), "AdapterNode::Main() for %s", this->get_name());
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(this->get_node_base_interface());
  executor.add_node(zivid_node_->get_node_base_interface());

  executor.spin();

  return absl::OkStatus();
}

absl::StatusOr<AdapterNode::CaptureData> AdapterNode::Capture() {
  RCLCPP_INFO(get_logger(), "Triggering a capture...");

  if (!capture_client_->service_is_ready()) {
    const std::string error_msg = "Capture service is not ready.";
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    return absl::UnavailableError(error_msg);
  }

  RCLCPP_INFO(get_logger(), "Sending capture request...");
  auto capture_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto capture_data = capture_client_->async_send_request(capture_request);

  auto future_status = capture_data.wait_for(std::chrono::seconds(10));
  if (future_status != std::future_status::ready) {
    const std::string error_msg = "Capture request timed out after 10 seconds.";
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    return absl::DeadlineExceededError(error_msg);
  }

  auto capture_response = capture_data.get();
  RCLCPP_INFO(get_logger(), "Capture request completed");

  if (!capture_response->success) {
    const std::string error_msg =
        absl::StrCat("Capture failed: ", capture_response->message);
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    return absl::InternalError(error_msg);
  }

  // Wait for all callbacks to complete with timeout
  constexpr absl::Duration timeout = absl::Seconds(10);
  absl::MutexLock lock(&data_mutex_);
  if (data_mutex_.AwaitWithTimeout(
          absl::Condition(&data_, &CaptureData::AllAvailable), timeout)) {
    RCLCPP_INFO(get_logger(), "Capture succeeded.");
    // Move all data out and cache only camera_info for future describe() calls
    CaptureData result{.color_image = std::move(data_.color_image),
                       .depth_image = std::move(data_.depth_image),
                       .normal_pc = std::move(data_.normal_pc),
                       .camera_info = data_.camera_info};
    return result;
  }

  std::vector<std::string> missing_items;
  if (!data_.color_image) missing_items.push_back("color image");
  if (!data_.depth_image) missing_items.push_back("depth image");
  if (!data_.normal_pc) missing_items.push_back("normals point cloud");
  if (!data_.camera_info) missing_items.push_back("camera info");

  const std::string error_msg = absl::StrCat(
      "Did not receive ", absl::StrJoin(missing_items, ", "),
      " from camera within timeout while waiting for capture data.");
  RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
  return absl::DeadlineExceededError(error_msg);
}

bool AdapterNode::BuildDescribeResponse(
    snapshot_interfaces::srv::Describe::Response& response) {
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_copy;

  {
    absl::MutexLock lock(&data_mutex_);
    info_copy = data_.camera_info;
  }

  if (info_copy == nullptr) {
    RCLCPP_INFO(get_logger(),
                "No cached camera_info available, triggering capture");
    auto capture_data = Capture();
    if (!capture_data.ok()) {
      response.error_message = std::string(capture_data.status().message());
      return false;
    }
    info_copy = capture_data->camera_info;
  }

  // Color sensor info
  AppendSensorDescription(response, *info_copy, "rgb", ColorImageTopic());
  // Depth sensor info
  AppendSensorDescription(response, *info_copy, "depth", DepthImageTopic());
  AppendSensorDescription(response, *info_copy, "normal", NormalTopic());

  return true;
}

bool AdapterNode::BuildSnapshotResponse(
    snapshot_interfaces::srv::Snapshot::Response& response) {
  // Trigger a capture
  auto capture_data = Capture();
  if (!capture_data.ok()) {
    response.error_message = std::string(capture_data.status().message());
    return false;
  }
  snapshot_interfaces::msg::ImageSnapshot color_snapshot;
  snapshot_interfaces::msg::ImageSnapshot depth_snapshot;
  snapshot_interfaces::msg::PointCloud2Snapshot normal_snapshot;

  color_snapshot.topic_name = ColorImageTopic();
  depth_snapshot.topic_name = DepthImageTopic();
  normal_snapshot.topic_name = NormalTopic();

  color_snapshot.camera_info = *capture_data->camera_info;
  depth_snapshot.camera_info = std::move(*capture_data->camera_info);

  color_snapshot.image = std::move(*capture_data->color_image);
  depth_snapshot.image = std::move(*capture_data->depth_image);
  normal_snapshot.point_cloud = std::move(*capture_data->normal_pc);

  response.images.push_back(std::move(color_snapshot));
  response.images.push_back(std::move(depth_snapshot));
  response.point_clouds.push_back(std::move(normal_snapshot));

  return true;
}
}  // namespace flowstate_zivid
