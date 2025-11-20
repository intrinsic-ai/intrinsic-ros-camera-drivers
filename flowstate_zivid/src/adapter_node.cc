#include "flowstate_zivid/adapter_node.h"

#include <Zivid/Settings.h>

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
#include "zivid_camera/capture_settings_controller.hpp"
#include "zivid_camera/utility.hpp"
#include "zivid_camera/zivid_camera.hpp"
namespace zivid_camera {
class ZividCamera;
}

namespace flowstate_zivid {

using snapshot_interfaces::srv::Describe;
using snapshot_interfaces::srv::Snapshot;

AdapterNode::AdapterNode(const std::string& serial,
                         const rclcpp::NodeOptions& options,
                         std::shared_ptr<Zivid::Application> zivid_app)
    : Node(std::string("zivid_") + serial, options),
      serial_(serial),
      capture_params_(ZividCaptureParameters::boot_defaults()) {
  // Declare parameters for AdapterNode
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

  // Declare FPS parameter for continuous capture
  declare_parameter<double>("fps", 0.0);

  // Declare parameters that will be passed through to zivid_camera node
  const std::string zivid_node_name = std::string("camera_") + serial;
  const std::string zivid_ns = "zivid/" + zivid_node_name;
  this->declare_parameter<std::string>("settings_yaml",
                                       "");  // For zivid_camera node

  this->declare_parameter<std::string>("settings_2d_yaml", "");
  this->declare_parameter<std::string>("settings_2d_file_path", "");
  this->declare_parameter<std::string>("color_space", "srgb");
  this->declare_parameter<std::string>("intrinsics_source", "camera");

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
          &AdapterNode::setParametersCallback, this, std::placeholders::_1));

  // Subscribe to color image + camera_info pair
  color_image_sub_ = image_transport::create_camera_subscription(
      this, ColorImageTopic(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& image,
             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
        {
          absl::MutexLock data_lock(&this->data_mutex_);
          this->color_image_ =
              std::make_unique<sensor_msgs::msg::Image>(*image);
          this->CheckAndClearCaptureFlag();
        }
        {
          absl::MutexLock info_lock(&this->camera_info_mutex_);
          this->color_camera_info_ =
              std::make_unique<sensor_msgs::msg::CameraInfo>(*camera_info);
        }
      },
      "raw");

  // Subscribe to depth image + camera_info pair
  depth_image_sub_ = image_transport::create_camera_subscription(
      this, DepthImageTopic(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& image,
             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
        {
          absl::MutexLock data_lock(&this->data_mutex_);
          this->depth_image_ =
              std::make_unique<sensor_msgs::msg::Image>(*image);
          this->CheckAndClearCaptureFlag();
        }
        {
          absl::MutexLock info_lock(&this->camera_info_mutex_);
          this->depth_camera_info_ =
              std::make_unique<sensor_msgs::msg::CameraInfo>(*camera_info);
        }
      },
      "raw");

  // Subscribe to normals point cloud
  normal_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      NormalTopic(), 2, [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
        {
          absl::MutexLock data_lock(&this->data_mutex_);
          this->normal_pc_ = std::move(msg);
          this->CheckAndClearCaptureFlag();
        }
      });

  callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  describe_service_ = create_service<Describe>(
      "~/describe",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<Describe::Request> request,
             const std::shared_ptr<Describe::Response> response) {
        this->DescribeCallback(request_header, request, response);
      },
      rclcpp::ServicesQoS(), callback_group_);

  snapshot_service_ = create_service<Snapshot>(
      "~/snapshot",
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<Snapshot::Request> request,
             const std::shared_ptr<Snapshot::Response> response) {
        this->SnapshotCallback(request_header, request, response);
      },
      rclcpp::ServicesQoS(), callback_group_);
  RCLCPP_INFO(this->get_logger(), "zivid_node_ = %s",
              zivid_node_->get_fully_qualified_name());

  capture_client_ = this->create_client<std_srvs::srv::Trigger>(
      absl::StrFormat("/zivid/camera_%s/capture", serial_.c_str()));

  thread_ = std::thread([this]() {
    const absl::Status status = this->Main();
    if (!status.ok()) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Adapter node thread error: " << status);
    }
    RCLCPP_INFO(this->get_logger(), "Destroying zivid_camera_node...");
    zivid_node_.reset();
    RCLCPP_INFO(this->get_logger(), "Done destroying zivid_camera_node.");
  });
}

rcl_interfaces::msg::SetParametersResult AdapterNode::setParametersCallback(
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
    } else if (param.get_name() == "fps") {
      const double fps = param.as_double();
      RCLCPP_INFO(get_logger(), "FPS parameter changed to: %.2f", fps);
      onCaptureTimer(fps);
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

std::string AdapterNode::CameraInfoTopic() const {
  return absl::StrFormat("/zivid/camera_%s/camera_info", serial_.c_str());
}

absl::Status AdapterNode::Main() {
  RCLCPP_INFO(get_logger(), "AdapterNode::Main() for %s", this->get_name());
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(this->get_node_base_interface());
  executor.add_node(zivid_node_->get_node_base_interface());

  executor.spin();

  return absl::OkStatus();
}

absl::Status AdapterNode::TriggerOnDemandCapture() {
  RCLCPP_INFO(get_logger(), "Triggering on-demand capture...");

  if (!capture_client_->service_is_ready()) {
    const std::string error_msg = "Capture service is not ready.";
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    return absl::UnavailableError(error_msg);
  }

  // Define lambda to check if all callbacks have completed
  auto all_callbacks_completed = [this]() {
    data_mutex_.AssertReaderHeld();
    return color_image_ != nullptr && depth_image_ != nullptr &&
           normal_pc_ != nullptr;
  };

  // Reset all data pointers to nullptr before triggering capture
  {
    absl::MutexLock lock(&data_mutex_);
    color_image_ = nullptr;
    depth_image_ = nullptr;
    normal_pc_ = nullptr;
    capture_in_progress_ = true;
  }

  RCLCPP_INFO(get_logger(), "Sending capture request...");
  auto capture_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto capture_result = capture_client_->async_send_request(capture_request);

  auto future_status = capture_result.wait_for(std::chrono::seconds(10));
  if (future_status != std::future_status::ready) {
    const std::string error_msg = "Capture request timed out after 10 seconds.";
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    return absl::DeadlineExceededError(error_msg);
  }

  auto capture_response = capture_result.get();
  RCLCPP_INFO(get_logger(), "Capture request completed");

  if (!capture_response->success) {
    const std::string error_msg =
        absl::StrCat("Capture failed: ", capture_response->message);
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    return absl::InternalError(error_msg);
  }

  // Wait for all callbacks to complete with timeout
  constexpr absl::Duration timeout = absl::Seconds(10);
  bool all_data_received = false;
  {
    absl::MutexLock lock(&data_mutex_);
    all_data_received = data_mutex_.AwaitWithTimeout(
        absl::Condition(&all_callbacks_completed), timeout);
  }

  if (!all_data_received) {
    std::vector<std::string> missing_items;
    {
      absl::MutexLock lock(&data_mutex_);
      capture_in_progress_ = false;
      if (!color_image_) missing_items.push_back("color image");
      if (!depth_image_) missing_items.push_back("depth image");
      if (!normal_pc_) missing_items.push_back("normals point cloud");
    }

    const std::string error_msg = absl::StrCat(
        "Did not receive ", absl::StrJoin(missing_items, ", "),
        " from camera within timeout while waiting for capture data.");
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    return absl::DeadlineExceededError(error_msg);
  }

  {
    absl::MutexLock lock(&data_mutex_);
    capture_in_progress_ = false;
  }

  RCLCPP_INFO(get_logger(), "On-demand capture succeeded.");
  return absl::OkStatus();
}

void AdapterNode::onCaptureTimer(double fps) {
  RCLCPP_INFO_STREAM(get_logger(), "FPS parameter is set to " << fps);

  // Always stop the existing timer if it's running before potentially starting
  // a new one.
  if (capture_timer_) {
    RCLCPP_INFO(get_logger(),
                "Stopping current continuous capture before (re)starting.");
    capture_timer_->cancel();
    capture_timer_.reset();
  }

  if (fps > 0.0) {
    const auto period = std::chrono::duration<double>(1.0 / fps);
    RCLCPP_INFO(get_logger(),
                "Starting continuous capture with a period of %.3f s (%.1f Hz)",
                period.count(), fps);

    capture_timer_ = this->create_wall_timer(period, [this]() {
      if (!capture_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000,
                             "Capture service is not ready.");
        return;
      }

      {
        absl::MutexLock lock(&data_mutex_);
        color_image_ = nullptr;
        depth_image_ = nullptr;
        normal_pc_ = nullptr;
        capture_in_progress_ = true;
      }

      capture_client_->async_send_request(
          std::make_shared<std_srvs::srv::Trigger::Request>());
    });
  } else {
    RCLCPP_INFO(get_logger(), "Continuous capture is disabled (fps <= 0.0).");
  }
}

absl::Status AdapterNode::WaitForOngoingCapture() {
  RCLCPP_INFO(get_logger(), "Waiting for ongoing capture to complete...");

  auto capture_completed = [this]() {
    data_mutex_.AssertReaderHeld();
    return !capture_in_progress_;
  };

  constexpr absl::Duration timeout = absl::Seconds(10);
  bool completed = false;
  {
    absl::MutexLock lock(&data_mutex_);
    completed = data_mutex_.AwaitWithTimeout(
        absl::Condition(&capture_completed), timeout);
  }

  if (!completed) {
    const std::string error_msg =
        "Timeout waiting for ongoing capture to complete.";
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    return absl::DeadlineExceededError(error_msg);
  }

  RCLCPP_INFO(get_logger(), "Ongoing capture completed.");
  return absl::OkStatus();
}

void AdapterNode::CheckAndClearCaptureFlag() {
  // Check if all data is now available and clear the flag
  if (color_image_ && depth_image_ && normal_pc_) {
    capture_in_progress_ = false;
  }
}

void AdapterNode::DescribeCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<snapshot_interfaces::srv::Describe::Request>,
    const std::shared_ptr<snapshot_interfaces::srv::Describe::Response>
        response) {
  RCLCPP_INFO(get_logger(), "=== DESCRIBE SERVICE ===");

  // Check if a capture is already in progress
  bool capture_ongoing = false;
  {
    absl::MutexLock lock(&data_mutex_);
    capture_ongoing = capture_in_progress_;
  }

  if (capture_ongoing) {
    // Wait for the ongoing capture to complete
    RCLCPP_INFO(get_logger(),
                "Capture already in progress, waiting for it to complete...");
    const absl::Status status = WaitForOngoingCapture();
    if (!status.ok()) {
      response->error_message = std::string(status.message());
      response->success = false;
      return;
    }
  } else {
    // Check if camera_info is available, if not trigger a capture
    bool need_capture = false;
    {
      absl::MutexLock lock(&camera_info_mutex_);
      need_capture = !color_camera_info_ || !depth_camera_info_;
    }

    if (need_capture) {
      RCLCPP_WARN(get_logger(),
                  "CameraInfo not yet received, triggering a capture...");
      const absl::Status status = TriggerOnDemandCapture();
      if (!status.ok()) {
        response->error_message = std::string(status.message());
        response->success = false;
        return;
      }
    }
  }

  // Color sensor info
  snapshot_interfaces::msg::SensorInfo color_info;
  color_info.sensor_name = "color";
  color_info.topic_name = ColorImageTopic();
  color_info.sensor_type = snapshot_interfaces::msg::SensorInfo::IMAGE;
  color_info.camera_t_sensor.transform.rotation.w = 1.0;
  {
    absl::MutexLock lock(&camera_info_mutex_);
    color_info.info.push_back(*color_camera_info_);
  }
  response->sensors.push_back(color_info);

  // Depth sensor info
  snapshot_interfaces::msg::SensorInfo depth_info;
  depth_info.sensor_name = "depth";
  depth_info.topic_name = DepthImageTopic();
  depth_info.sensor_type = snapshot_interfaces::msg::SensorInfo::DEPTH;
  depth_info.camera_t_sensor.transform.rotation.w = 1.0;
  {
    absl::MutexLock lock(&camera_info_mutex_);
    depth_info.info.push_back(*depth_camera_info_);
  }
  response->sensors.push_back(depth_info);

  snapshot_interfaces::msg::SensorInfo normal_info;
  normal_info.sensor_name = "normal";
  normal_info.topic_name = NormalTopic();
  normal_info.sensor_type = snapshot_interfaces::msg::SensorInfo::NORMAL;
  normal_info.camera_t_sensor.transform.rotation.w = 1.0;
  response->sensors.push_back(normal_info);

  response->success = true;
}

void AdapterNode::SnapshotCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Request>,
    const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Response>
        response) {
  RCLCPP_INFO(get_logger(), "=== SNAPSHOT SERVICE ===");

  // Check if a capture is already in progress
  bool capture_ongoing = false;
  {
    absl::MutexLock lock(&data_mutex_);
    capture_ongoing = capture_in_progress_;
  }

  if (capture_ongoing) {
    // Wait for the ongoing capture to complete
    RCLCPP_INFO(get_logger(),
                "Capture already in progress, waiting for it to complete...");
    const absl::Status status = WaitForOngoingCapture();
    if (!status.ok()) {
      response->error_message = std::string(status.message());
      response->success = false;
      return;
    }
  } else {
    // Trigger an on-demand capture
    const absl::Status status = TriggerOnDemandCapture();
    if (!status.ok()) {
      response->error_message = std::string(status.message());
      response->success = false;
      return;
    }
  }

  snapshot_interfaces::msg::ImageSnapshot color_snapshot;
  snapshot_interfaces::msg::ImageSnapshot depth_snapshot;
  snapshot_interfaces::msg::PointCloud2Snapshot normal_snapshot;

  color_snapshot.topic_name = ColorImageTopic();
  depth_snapshot.topic_name = DepthImageTopic();
  normal_snapshot.topic_name = NormalTopic();

  {
    absl::MutexLock info_lock(&camera_info_mutex_);
    absl::MutexLock data_lock(&data_mutex_);

    color_snapshot.camera_info = *color_camera_info_;
    depth_snapshot.camera_info = *depth_camera_info_;

    color_snapshot.image = *color_image_;
    depth_snapshot.image = *depth_image_;
    normal_snapshot.point_cloud = *normal_pc_;
  }

  response->images.push_back(std::move(color_snapshot));
  response->images.push_back(std::move(depth_snapshot));
  response->point_clouds.push_back(std::move(normal_snapshot));

  response->success = true;
}
}  // namespace flowstate_zivid
