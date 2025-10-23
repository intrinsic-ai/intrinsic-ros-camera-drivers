// Modified from https://github.com/zivid/zivid-ros/blob/2226e6d48b0326097ee0dbffe50a10c14f5f3881/zivid_camera/src/zivid_camera.cpp
// Added support for flowstate ros camera services

#include <Zivid/Application.h>
#include <Zivid/Camera.h>
#include <Zivid/CaptureAssistant.h>
#include <Zivid/Exception.h>
#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Experimental/PointCloudExport.h>
#include <Zivid/Firmware.h>
#include <Zivid/Frame2D.h>
#include <Zivid/Image.h>
#include <Zivid/Settings2D.h>
#include <Zivid/Experimental/SettingsInfo.h>
#include <Zivid/Version.h>

#include <algorithm>
#include <cstdint>
#include <map>
#include <numeric>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <zivid_camera/capture_settings_controller.hpp>
#include <zivid_camera/detector_controller.hpp>
#include <zivid_camera/hand_eye_calibration_controller.hpp>
#include <zivid_camera/infield_correction_controller.hpp>
#include <zivid_camera/projection_controller.hpp>
#include <zivid_camera/utility.hpp>
#include "zivid_camera_node.h"

// snapshot_interfaces
#include "snapshot_interfaces/srv/describe.hpp"
#include "snapshot_interfaces/msg/sensor_info.hpp"
#include "snapshot_interfaces/srv/snapshot.hpp"
#include "snapshot_interfaces/msg/image_snapshot.hpp"
#include "snapshot_interfaces/msg/point_cloud2_snapshot.hpp"
namespace
{
sensor_msgs::msg::PointField createPointField(
  std::string name, uint32_t offset, uint8_t datatype, uint32_t count)
{
  sensor_msgs::msg::PointField point_field;
  point_field.name = name;
  point_field.offset = offset;
  point_field.datatype = datatype;
  point_field.count = count;
  return point_field;
}

bool bigEndian()
{
  union {
    uint32_t i;
    char c[4];
  } b = {0x01020304};
  return b.c[0] == 1;
}

template <class T>
void fillCommonMsgFields(
  T & msg, const std_msgs::msg::Header & header, std::size_t width, std::size_t height)
{
  msg.header = header;
  msg.height = static_cast<uint32_t>(height);
  msg.width = static_cast<uint32_t>(width);
  msg.is_bigendian = bigEndian();
}

sensor_msgs::msg::Image::SharedPtr makeImage(
  const std_msgs::msg::Header & header, const std::string & encoding, std::size_t width,
  std::size_t height)
{
  auto image = std::make_shared<sensor_msgs::msg::Image>();
  fillCommonMsgFields(*image, header, width, height);
  image->encoding = encoding;
  const auto bytes_per_pixel = static_cast<std::size_t>(
    sensor_msgs::image_encodings::numChannels(encoding) *
    sensor_msgs::image_encodings::bitDepth(encoding) / 8);
  image->step = static_cast<uint32_t>(bytes_per_pixel * width);
  return image;
}

template <typename ZividDataType>
sensor_msgs::msg::Image::SharedPtr makePointCloudImage(
  const Zivid::PointCloud & point_cloud, const std_msgs::msg::Header & header,
  const std::string & encoding)
{
  auto image = makeImage(header, encoding, point_cloud.width(), point_cloud.height());
  image->data.resize(image->step * image->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType *>(image->data.data()));
  return image;
}

template <typename ZividDataType>
sensor_msgs::msg::Image::SharedPtr makeImageFromZividImage(
  const Zivid::Image<ZividDataType> & image, const std_msgs::msg::Header & header,
  const std::string & encoding)
{
  auto msg = makeImage(header, encoding, image.width(), image.height());
  const auto uint8_ptr_begin = reinterpret_cast<const uint8_t *>(image.data());

#ifdef __clang__
#if __has_warning("-Wunsafe-buffer-usage")
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage"
#endif
#endif
  const auto uint8_ptr_end = reinterpret_cast<const uint8_t *>(image.data() + image.size());
#ifdef __clang__
#if __has_warning("-Wunsafe-buffer-usage")
#pragma clang diagnostic pop
#endif
#endif

  msg->data = std::vector<uint8_t>(uint8_ptr_begin, uint8_ptr_end);
  return msg;
}

template <typename ZividDataType>
sensor_msgs::msg::PointCloud2::UniquePtr makePointCloud2Msg(
  const Zivid::PointCloud & point_cloud, const std_msgs::msg::Header & header)
{
  // Note that the "rgba" field is actually byte order "bgra" on little-endian systems. For this
  // reason we use the Zivid BGRA type.
  static_assert(
    std::is_same_v<ZividDataType, Zivid::PointXYZColorBGRA> ||
    std::is_same_v<ZividDataType, Zivid::PointXYZColorBGRA_SRGB>);

  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->fields.reserve(4);
  msg->fields.push_back(createPointField("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("rgba", 12, sensor_msgs::msg::PointField::UINT32, 1));
  msg->is_dense = false;

  msg->point_step = sizeof(ZividDataType);
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step * msg->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType *>(msg->data.data()));
  return msg;
}

rclcpp::QoS getQoSLatched(bool use_latched_publisher)
{
  rclcpp::QoS qos{rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)};
  if (use_latched_publisher) {
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  }
  return qos;
}

std::string toString(zivid_camera_node::CameraStatus camera_status)
{
  switch (camera_status) {
    case zivid_camera_node::CameraStatus::Connected:
      return "Connected";
    case zivid_camera_node::CameraStatus::Disconnected:
      return "Disconnected";
    case zivid_camera_node::CameraStatus::Idle:
      return "Idle";
    default:  // NOLINT(clang-diagnostic-covered-switch-default)
      throw std::runtime_error("Enum `camera_status` out of range.");
  }
}

template <typename List, typename ToStringFunc>
std::string joinListToString(const List & list, ToStringFunc && to_string_func)
{
  return list.empty()
           ? std::string()
           : std::accumulate(
               std::next(std::begin(list)), std::end(list), to_string_func(*std::begin(list)),
               [&](const std::string & str, const auto & entry) {
                 return str + ", " + to_string_func(entry);
               });
}

template <typename Enum>
Enum parameterStringToEnum(
  const std::string & name, const std::string & value,
  const std::map<std::string, Enum> & name_value_map)
{
  const auto it = name_value_map.find(value);
  if (it == name_value_map.end()) {
    const std::string valid_values =
      joinListToString(name_value_map, [](auto && pair) { return pair.first; });
    throw std::runtime_error(
      "Invalid value for parameter '" + name + "': '" + value +
      "'. Expected one of: " + valid_values + ".");
  }
  return it->second;
}

}  // namespace

namespace zivid_camera_node
{
using ::snapshot_interfaces::srv::Describe;
using ::snapshot_interfaces::srv::Snapshot;

namespace ParamNames
{
constexpr auto serial_number = "serial_number";
constexpr auto file_camera_path = "file_camera_path";
constexpr auto frame_id = "frame_id";
constexpr auto color_space = "color_space";
constexpr auto intrinsics_source = "intrinsics_source";
}  // namespace ParamNames

Zivid::Settings ZividCamNode::generateZividSettings() const
{
  std::lock_guard<std::mutex> lock(capture_params_mutex_);
  std::stringstream yaml;
  yaml << "__version__:\n"
       << "  serializer: 1\n"
       << "  data: 22\n"
       << "Settings:\n"
       << "  Acquisitions:\n";

  auto add_acquisition_to_yaml = [&](double exposure_seconds) {
    yaml << "    - Acquisition:\n"
         << "        Aperture: " << capture_params_.aperture << "\n"
         << "        Brightness: " << capture_params_.projector_brightness << "\n"
         << "        ExposureTime: " << static_cast<long long>(exposure_seconds) << "\n"
         << "        Gain: " << capture_params_.gain << "\n";
  };
  // * 1000000 to convert seconds to microseconds

  add_acquisition_to_yaml(capture_params_.exposure_time);

  yaml << "  Processing:\n"
       << "    Color:\n"
       << "      Gamma: " << capture_params_.gamma << "\n"
       << "    Filters:\n"
       << "      Outlier:\n"
       << "        Removal:\n"
       << "          Enabled: " << (capture_params_.outlier_removal_enabled ? "yes" : "no") << "\n"
       << "          Threshold: " << capture_params_.outlier_removal_threshold << "\n";

  const auto settings_yaml = yaml.str();
  // RCLCPP_DEBUG_STREAM(get_logger(), "Generated settings yaml:\n" << settings_yaml);
  return Zivid::Settings::fromSerialized(settings_yaml);
}

void ZividCamNode::updateSettingsYamlCallback()
{
  // Check if the flag is set, and if it is, atomically set it to false.
  if (individual_settings_dirty_.exchange(false))
  {
    RCLCPP_INFO(get_logger(), "Asynchronously updating settings_yaml from individual parameters.");
    const auto settings = generateZividSettings();
    if (settings_controller_) {
      settings_controller_->setSettings(settings);
    }
  }
}

// Constructor that accepts a pre-created camera (avoids duplicate file camera creation)
ZividCamNode::ZividCamNode(const std::string & node_name, const rclcpp::NodeOptions & options, Zivid::Camera & camera, Zivid::Application & zivid_app)
: rclcpp::Node(node_name, options),  // Use the provided node name with options
  color_space_name_value_map_{
    {"srgb", zivid_camera_node::ColorSpace::sRGB},
    {"linear_rgb", zivid_camera_node::ColorSpace::LinearRGB},
  },
  intrinsics_source_name_value_map_{
    {"camera", zivid_camera_node::IntrinsicsSource::Camera},
    {"frame", zivid_camera_node::IntrinsicsSource::Frame},
  },
  zivid_app_ref_(zivid_app),  // Use the provided application reference
  set_parameters_callback_handle_{this->add_on_set_parameters_callback(
    std::bind(&ZividCamNode::setParametersCallback, this, std::placeholders::_1))}
{
  declare_parameter<std::string>(ParamNames::color_space, "linear_rgb");
  declare_parameter<std::string>(ParamNames::intrinsics_source, "camera");

  // Declare Zivid-specific capture parameters with boot defaults
  capture_params_ = ZividCaptureParameters::boot_defaults();
  declare_parameter<double>("framerate", capture_params_.framerate);
  declare_parameter<double>("exposure_time", capture_params_.exposure_time);
  declare_parameter<double>("ExposureTime", capture_params_.exposure_time);
  declare_parameter<double>("gain", capture_params_.gain);
  declare_parameter<double>("Gain", capture_params_.gain);
  declare_parameter<double>("gamma", capture_params_.gamma);
  declare_parameter<double>("Gamma", capture_params_.gamma);

  declare_parameter<double>("projector_brightness", capture_params_.projector_brightness);
  declare_parameter<double>("brightness", capture_params_.projector_brightness);
  declare_parameter<double>("Brightness", capture_params_.projector_brightness);

  declare_parameter<double>("aperture", capture_params_.aperture);
  declare_parameter<double>("Aperture", capture_params_.aperture);

  declare_parameter<bool>("outlier_removal_enabled", capture_params_.outlier_removal_enabled);
  declare_parameter<double>("outlier_removal_threshold", capture_params_.outlier_removal_threshold);

  // Use the provided camera directly instead of creating a new one
  camera_ = std::make_unique<Zivid::Camera>(camera);

  // Initialize both settings controllers
  settings_controller_ = std::make_unique<zivid_camera::CaptureSettingsController<Zivid::Settings>>(*this);

  // Log camera info
  RCLCPP_INFO_STREAM(get_logger(), *camera_);

  // Connect to camera only if not already connected
  if (camera_->state().isConnected().value()) {
    RCLCPP_INFO_STREAM(
      get_logger(), "Camera '" << camera_->info().serialNumber() << "' is already connected");
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "Connecting to camera ...");
    camera_->connect();
    RCLCPP_INFO_STREAM(
      get_logger(), "Connected to camera '" << camera_->info().serialNumber() << "'");
  }
  setCameraStatus(CameraStatus::Connected);

  init();

  // Start a timer to periodically check if the settings_yaml needs to be updated.
  using namespace std::chrono_literals;
  update_settings_yaml_timer_ =
    create_wall_timer(100ms, std::bind(&ZividCamNode::updateSettingsYamlCallback, this));
}

ZividCamNode::~ZividCamNode() = default;

Zivid::Application & ZividCamNode::zividApplication() { return zivid_app_ref_; }

void ZividCamNode::init()
{
  // Disable buffering on stdout
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  RCLCPP_INFO_STREAM(get_logger(), "Running Zivid Core version " << ZIVID_CORE_VERSION);

  // The settings_controller_ is replaced by individual parameters.
  settings_2d_controller_ = std::make_unique<zivid_camera::CaptureSettingsController<Zivid::Settings2D>>(*this);

  frame_id_ = declare_parameter<std::string>(ParamNames::frame_id, "zivid_optical_frame");

  use_latched_publisher_for_points_xyz_ =
    declare_parameter<bool>("use_latched_publisher_for_points_xyz", false);
  use_latched_publisher_for_points_xyzrgba_ =
    declare_parameter<bool>("use_latched_publisher_for_points_xyzrgba", false);
  use_latched_publisher_for_color_image_ =
    declare_parameter<bool>("use_latched_publisher_for_color_image", false);
  use_latched_publisher_for_depth_image_ =
    declare_parameter<bool>("use_latched_publisher_for_depth_image", false);
  use_latched_publisher_for_snr_image_ =
    declare_parameter<bool>("use_latched_publisher_for_snr_image", false);
  use_latched_publisher_for_normals_xyz_ =
    declare_parameter<bool>("use_latched_publisher_for_normals_xyz", false);

  points_xyz_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/points/xyz", getQoSLatched(use_latched_publisher_for_points_xyz_));

  points_xyzrgba_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/points/xyzrgba", getQoSLatched(use_latched_publisher_for_points_xyzrgba_));

  normals_xyz_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/normals/xyz", getQoSLatched(use_latched_publisher_for_normals_xyz_));

  color_image_publisher_ = image_transport::create_camera_publisher(
    this, "~/color/image_color",
    getQoSLatched(use_latched_publisher_for_color_image_).get_rmw_qos_profile());
  depth_image_publisher_ = image_transport::create_camera_publisher(
    this, "~/depth/image",
    getQoSLatched(use_latched_publisher_for_depth_image_).get_rmw_qos_profile());
  snr_image_publisher_ = image_transport::create_camera_publisher(
    this, "~/snr/image", getQoSLatched(use_latched_publisher_for_snr_image_).get_rmw_qos_profile());

  RCLCPP_INFO(get_logger(), "Advertising services");

  using namespace std::placeholders;

  camera_info_model_name_service_ = create_service<zivid_interfaces::srv::CameraInfoModelName>(
    "~/camera_info/model_name", std::bind(&ZividCamNode::cameraInfoModelNameServiceHandler, this, _1, _2, _3));

  camera_info_serial_number_service_ = create_service<zivid_interfaces::srv::CameraInfoSerialNumber>(
    "~/camera_info/serial_number", std::bind(&ZividCamNode::cameraInfoSerialNumberServiceHandler, this, _1, _2, _3));

  is_connected_service_ = create_service<zivid_interfaces::srv::IsConnected>(
    "~/is_connected", std::bind(&ZividCamNode::isConnectedServiceHandler, this, _1, _2, _3));

  capture_service_ =
    create_service<Snapshot>("~/snapshot", std::bind(&ZividCamNode::captureServiceHandler, this, _1, _2, _3));

  capture_and_save_service_ = create_service<zivid_interfaces::srv::CaptureAndSave>(
    "~/capture_and_save", std::bind(&ZividCamNode::captureAndSaveServiceHandler, this, _1, _2, _3));

  capture_2d_service_ = create_service<std_srvs::srv::Trigger>(
    "~/capture_2d", std::bind(&ZividCamNode::capture2DServiceHandler, this, _1, _2, _3));

  capture_assistant_suggest_settings_service_ = create_service<zivid_interfaces::srv::CaptureAssistantSuggestSettings>(
    "~/capture_assistant/suggest_settings", std::bind(&ZividCamNode::captureAssistantSuggestSettingsServiceHandler, this, _1, _2, _3));

  describe_service_ = create_service<Describe>("~/describe", std::bind(&ZividCamNode::describe_callback, this, _1, _2, _3));

  RCLCPP_INFO(get_logger(), "Zivid camera driver for %s is now ready!", get_name());
}

void ZividCamNode::describe_callback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<Describe::Request>,
    const std::shared_ptr<Describe::Response>
        response) {
  try {
    // Common information for all sensors
    RCLCPP_INFO_STREAM(get_logger(), "============Describe service called.============");

    const auto node_name = std::string(get_fully_qualified_name());

    auto header = makeHeader();  // This sets timestamp and frame_id

    auto intrinsics = Zivid::Experimental::Calibration::intrinsics(*camera_);
    // Get image dimensions from camera info
    Zivid::Settings settings;
    try {
      settings = settings_controller_->currentSettings();
      RCLCPP_INFO_STREAM(get_logger(), "got 3d settings for describe service.");

    } catch (const std::runtime_error &) {
      RCLCPP_INFO_STREAM(get_logger(), "No 3D settings configured, using default settings for describe service.");
      // settings is already default-constructed, so we just proceed.
    }
    const auto resolution = Zivid::Experimental::SettingsInfo::resolution(camera_->info(), settings);

    const uint32_t image_width = resolution.width();
    const uint32_t image_height = resolution.height();
    auto camera_info = makeCameraInfo(header, image_width, image_height, intrinsics);

    // The transform from the camera's optical frame to itself is identity.
    // The snapshot interface expects a transform from the camera's base link,
    // but this node is not aware of the base link frame or the transform.
    // We provide an identity transform relative to the optical frame.
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = get_clock()->now();
    transform.header.frame_id = frame_id_;  // e.g., "zivid_optical_frame"
    transform.child_frame_id = frame_id_;   // e.g., "zivid_optical_frame"
    transform.transform.rotation.w = 1.0;
    // The snapshot interface expects a transform from the camera's base link,
    // Color Image Sensor
    {
      snapshot_interfaces::msg::SensorInfo sensor_info;
      sensor_info.sensor_name = "color";
      sensor_info.topic_name = node_name + "/color/image_color";
      sensor_info.sensor_type = snapshot_interfaces::msg::SensorInfo::IMAGE;
      sensor_info.camera_t_sensor = transform;
      sensor_info.info.push_back(*camera_info);
      response->sensors.push_back(sensor_info);
    }
    {
      snapshot_interfaces::msg::SensorInfo sensor_info;
      sensor_info.sensor_name = "depth";
      sensor_info.topic_name = node_name + "/depth/image";
      sensor_info.sensor_type = snapshot_interfaces::msg::SensorInfo::DEPTH;
      sensor_info.camera_t_sensor = transform;
      sensor_info.info.push_back(*camera_info);
      response->sensors.push_back(sensor_info);
    }
    // Normals Sensor
    {
      snapshot_interfaces::msg::SensorInfo sensor_info;
      sensor_info.sensor_name = "normals";
      sensor_info.topic_name = node_name + "/normals/xyz";
      sensor_info.sensor_type = snapshot_interfaces::msg::SensorInfo::NORMAL;
      sensor_info.camera_t_sensor = transform;
      sensor_info.info.push_back(*camera_info);
      response->sensors.push_back(sensor_info);
    }
    response->success = true;
    RCLCPP_INFO_STREAM(get_logger(), "Describe service succeeded.");
  } catch (const std::exception& e) {
    response->success = false;
    response->error_message = e.what();
    RCLCPP_ERROR(get_logger(), "Describe service failed: %s", e.what());
    reconnectToCameraIfNecessary();
  }
}

void ZividCamNode::reconnectToCameraIfNecessary()
{
  RCLCPP_DEBUG_STREAM(get_logger(), __func__);

  const auto state = camera_->state();
  if (state.isConnected().value()) {
    setCameraStatus(CameraStatus::Connected);
  } else {
    setCameraStatus(CameraStatus::Disconnected);

    if (state.isAvailable().value()) {
      RCLCPP_INFO_STREAM(
        get_logger(), "The camera '" << camera_->info().serialNumber()
                                     << "' is not connected but is available. Re-connecting ...");
      camera_->connect();
      RCLCPP_INFO(get_logger(), "Successfully reconnected to camera!");
      setCameraStatus(CameraStatus::Connected);
    } else {
      RCLCPP_INFO_STREAM(
        get_logger(),
        "The camera '" << camera_->info().serialNumber() << "' is not connected nor available.");
    }
  }
}

void ZividCamNode::setCameraStatus(CameraStatus camera_status)
{
  if (camera_status_ != camera_status) {
    std::stringstream ss;
    ss << "Camera status changed to " << toString(camera_status) << " (was "
       << toString(camera_status_) << ")";
    if (camera_status == CameraStatus::Connected) {
      RCLCPP_INFO_STREAM(get_logger(), ss.str());
    } else {
      RCLCPP_WARN_STREAM(get_logger(), ss.str());
    }
    camera_status_ = camera_status;
  }
}

rcl_interfaces::msg::SetParametersResult ZividCamNode::setParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  bool individual_param_was_set = false;

  // If settings_yaml or settings_file_path is set in this batch, it takes precedence.
  // We should not schedule an update from individual parameters in this case.
  bool yaml_was_set = false;
  for (const auto & param : parameters) {
    if ((param.get_name() == "settings_yaml" || param.get_name() == "settings_file_path") &&
        !param.as_string().empty()) {
      yaml_was_set = true;
      break;
    }
  }

  std::lock_guard<std::mutex> lock(capture_params_mutex_);
  for (const auto & param : parameters) {
    RCLCPP_INFO_STREAM(
      get_logger(), "Setting parameter '" << param.get_name() << "' (" << param.get_type_name()
                                      << ") to '" << param.value_to_string() << "'");

    bool param_is_individual = true;
    if (param.get_name() == "exposure_time"|| param.get_name() == "ExposureTime") {
      RCLCPP_INFO_STREAM(get_logger(), "Exposure time needs to have microseconds as unit in Zivid API.");
      const double exposure_seconds = param.as_double();
      capture_params_.exposure_time = exposure_seconds;
    } else if (param.get_name() == "gain"|| param.get_name() == "Gain") {
      capture_params_.gain = param.as_double();
    } else if (param.get_name() == "gamma"|| param.get_name() == "Gamma") {
      capture_params_.gamma = param.as_double();
    } else if (param.get_name() == "projector_brightness"||param.get_name() == "brightness"|| param.get_name() == "Brightness" ) {
      capture_params_.projector_brightness = param.as_double();
    } else if (param.get_name() == "aperture"|| param.get_name() == "Aperture") {
      capture_params_.aperture = param.as_double();
    } else if (param.get_name() == "outlier_removal_enabled"|| param.get_name() == "OutlierRemovalEnabled") {
      capture_params_.outlier_removal_enabled = param.as_bool();
    } else if (param.get_name() == "outlier_removal_threshold"|| param.get_name() == "OutlierRemovalThreshold") {
      capture_params_.outlier_removal_threshold = param.as_double();
    } else {
      param_is_individual = false;
    }

    if (param_is_individual) {
      individual_param_was_set = true;
    } else {
      if (settings_controller_) {
        settings_controller_->onSetParameter(param.get_name());
      }
      if (settings_2d_controller_) {
        settings_2d_controller_->onSetParameter(param.get_name());
      }
    }
  }

  if (yaml_was_set) {
    // If yaml was set, it takes precedence. Don't schedule an update from individual params.
    individual_settings_dirty_ = false;
  } else if (individual_param_was_set) {
    // If only individual params were set, schedule an update.
    RCLCPP_INFO_STREAM(get_logger(), "Individual capture parameter changed, scheduling settings_yaml update.");
    individual_settings_dirty_ = true;
  }
  return result;
}

void ZividCamNode::cameraInfoModelNameServiceHandler(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Request>,
  std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);
  response->model_name = camera_->info().modelName().toString();
}

void ZividCamNode::cameraInfoSerialNumberServiceHandler(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Request>,
  std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);
  response->serial_number = camera_->info().serialNumber().toString();
}

void ZividCamNode::captureServiceHandler(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Request> /*request*/,
  std::shared_ptr<snapshot_interfaces::srv::Snapshot::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), "============Service capture called.============");

  try
  {

    serviceHandlerHandleCameraConnectionLoss();
    const auto settings = settings_controller_->currentSettings();
    RCLCPP_INFO_STREAM(get_logger(), "Got 3d settings for capture service.");
    const auto frame = camera_->capture(settings);
    // The point cloud in the frame will be modified by the mm-to-m scaling transform.
    // To avoid issues with double-transformations, we first populate the service
    // response, which performs the transform. Then, we call publishFrame, which will
    // see the already-transformed point cloud and skip its own transformation.
    populateSnapshotResponse(response, frame);
    // Also publish the data to the standard topics.
    publishFrame(frame);
    RCLCPP_INFO_STREAM(get_logger(), "Capture service succeeded.");
  }
  catch (const std::exception & e)
  {
    response->success = false;
    response->error_message = e.what();
    RCLCPP_ERROR(get_logger(), "Capture service failed: %s", e.what());
  }
}

void ZividCamNode::populateSnapshotResponse(
  std::shared_ptr<snapshot_interfaces::srv::Snapshot::Response> response,
  const Zivid::Frame & frame)
{
    const auto color_space = colorSpace();
    const auto header = makeHeader();
    auto point_cloud = frame.pointCloud();
    zivid_camera::ensureIdentityOrThrow(point_cloud.transformationMatrix());
    // Transform point cloud from millimeters (Zivid SDK) to meter (ROS).
    const float scale = 0.001f;
    const auto transformation_matrix =
      Zivid::Matrix4x4{scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, 1};
    point_cloud.transform(transformation_matrix);
    // Create and add PointCloud2 snapshot
    {
      snapshot_interfaces::msg::PointCloud2Snapshot normals_snapshot;
      normals_snapshot.topic_name = std::string(get_fully_qualified_name()) + "/normals/xyz";

      using ZividDataType = Zivid::NormalXYZ;
      auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
      fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
      msg->fields.reserve(3);
      msg->fields.push_back(createPointField("normal_x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
      msg->fields.push_back(createPointField("normal_y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
      msg->fields.push_back(createPointField("normal_z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
      msg->is_dense = false;
      msg->point_step = sizeof(ZividDataType);
      msg->row_step = msg->point_step * msg->width;
      msg->data.resize(msg->row_step * msg->height);
      point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType *>(msg->data.data()));
      normals_snapshot.point_cloud = *msg;
      response->point_clouds.push_back(normals_snapshot);
      RCLCPP_INFO_STREAM(get_logger(), "Added normals point cloud to snapshot response");
    }


    // Also add the images to the snapshot response
    auto intrinsics = [&] {
      const auto intrinsics_source = intrinsicsSource();
      switch (intrinsics_source) {
        case IntrinsicsSource::Camera:
          return Zivid::Experimental::Calibration::intrinsics(*camera_);
        case IntrinsicsSource::Frame:
          return Zivid::Experimental::Calibration::estimateIntrinsics(frame);
        default:
          throw std::runtime_error(
            "Internal error: Unknown intrinsics source value " +
            std::to_string(static_cast<int>(intrinsics_source)));
      }
    }();
    const auto camera_info = makeCameraInfo(header, point_cloud.width(), point_cloud.height(), intrinsics);
    // Color Image
    {
      snapshot_interfaces::msg::ImageSnapshot image_snapshot;
      image_snapshot.topic_name = std::string(get_fully_qualified_name()) + "/color/image_color";
      image_snapshot.camera_info = *camera_info;
      sensor_msgs::msg::Image::SharedPtr image = [&] {
        switch (color_space) {
          case ColorSpace::sRGB:
            RCLCPP_INFO_STREAM(get_logger(), "Creating color image RGBA8 for sRGB");
            return makePointCloudImage<Zivid::ColorRGBA_SRGB>(
              point_cloud, header, sensor_msgs::image_encodings::RGBA8);
          case ColorSpace::LinearRGB:
            RCLCPP_INFO_STREAM(get_logger(), "Creating color image RGBA8 for LinearRGB");
            return makePointCloudImage<Zivid::ColorRGBA>(
              point_cloud, header, sensor_msgs::image_encodings::RGBA8);
          default:
            throw std::runtime_error(
              "Internal error: Unknown color space value " +
              std::to_string(static_cast<int>(color_space)));
        }
      }();
      image_snapshot.image = *image;
      response->images.push_back(image_snapshot);
      RCLCPP_INFO_STREAM(get_logger(), "Added color image to snapshot response");
    }

    // Depth Image
    {
      snapshot_interfaces::msg::ImageSnapshot image_snapshot;
      image_snapshot.topic_name = std::string(get_fully_qualified_name()) + "/depth/image";
      image_snapshot.camera_info = *camera_info;
      auto image = makePointCloudImage<Zivid::PointZ>(
        point_cloud, header, sensor_msgs::image_encodings::TYPE_32FC1);
      image_snapshot.image = *image;
      response->images.push_back(image_snapshot);
      RCLCPP_INFO_STREAM(get_logger(), "Added depth image to snapshot response");
    }

    response->success = true;
    RCLCPP_INFO_STREAM(get_logger(), "Populated snapshot response successfully");
}

void ZividCamNode::captureAndSaveServiceHandler(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<zivid_interfaces::srv::CaptureAndSave::Request> request,
  std::shared_ptr<zivid_interfaces::srv::CaptureAndSave::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  zivid_camera::runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      const auto settings = settings_controller_->currentSettings();
      const auto frame = invokeCaptureAndPublishFrame(settings);
      RCLCPP_INFO(get_logger(), "Saving frame to '%s'", request->file_path.c_str());
      exportFrame(frame, request->file_path, colorSpace());
    },
    response, get_logger(), "CaptureAndSave");
}

void ZividCamNode::capture2DServiceHandler(
  const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  serviceHandlerHandleCameraConnectionLoss();

  zivid_camera::runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      const auto settings2D = settings_2d_controller_->currentSettings();
      auto frame2D = camera_->capture(settings2D);
      publishFrame2D(frame2D);
    },
    response, get_logger(), "Capture2D");
}

void ZividCamNode::captureAssistantSuggestSettingsServiceHandler(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request> request,
  std::shared_ptr<zivid_interfaces::srv::CaptureAssistantSuggestSettings::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  serviceHandlerHandleCameraConnectionLoss();

  zivid_camera::runFunctionAndCatchExceptionsForTriggerResponse(
    [&]() {
      using SuggestSettingsParameters = Zivid::CaptureAssistant::SuggestSettingsParameters;

      const auto max_capture_time =
        rclcpp::Duration{request->max_capture_time}.to_chrono<std::chrono::milliseconds>();
      const auto ambient_light_frequency = [this, &request]() {
        using RosRequestTypes = zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request;
        switch (request->ambient_light_frequency) {
          case RosRequestTypes::AMBIENT_LIGHT_FREQUENCY_NONE:
            return SuggestSettingsParameters::AmbientLightFrequency::none;
          case RosRequestTypes::AMBIENT_LIGHT_FREQUENCY_50HZ:
            return SuggestSettingsParameters::AmbientLightFrequency::hz50;
          case RosRequestTypes::AMBIENT_LIGHT_FREQUENCY_60HZ:
            return SuggestSettingsParameters::AmbientLightFrequency::hz60;
          default:
            logErrorAndThrowRuntimeException(
              "Unhandled AMBIENT_LIGHT_FREQUENCY value: " +
              std::to_string(request->ambient_light_frequency));
        }
      }();

      SuggestSettingsParameters suggest_settings_parameters{
        SuggestSettingsParameters::MaxCaptureTime{max_capture_time}, ambient_light_frequency};

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Getting suggested settings using parameters: " << suggest_settings_parameters);

      const auto suggested_settings =
        Zivid::CaptureAssistant::suggestSettings(*camera_, suggest_settings_parameters);

      RCLCPP_INFO_STREAM(
        get_logger(), "CaptureAssistant::suggestSettings returned "
                        << suggested_settings.acquisitions().size() << " acquisitions");

      // The settings_controller_ is replaced. We cannot set settings from CA this way anymore.
      // A more complex implementation would be needed to parse suggested_settings and update individual params.
      // For now, this functionality is disabled when using individual parameters.
      // settings_controller_->setSettings(suggested_settings);
      response->suggested_settings = zivid_camera::serializeZividDataModel(suggested_settings);
    },
    response, get_logger(), "CaptureAssistantSuggestSettings");
}

void ZividCamNode::serviceHandlerHandleCameraConnectionLoss()
{
  RCLCPP_INFO_STREAM(get_logger(), "Camera connection lost");
  reconnectToCameraIfNecessary();
  RCLCPP_INFO_STREAM(get_logger(), "Camera reconnection attempted");
  if (camera_status_ != CameraStatus::Connected) {
    logErrorAndThrowRuntimeException(
      "Unable to capture since the camera is not connected. Please re-connect the camera and "
      "try again.");
  }
  RCLCPP_INFO_STREAM(get_logger(), "Camera connection restored");
}

void ZividCamNode::isConnectedServiceHandler(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<zivid_interfaces::srv::IsConnected::Request>,
  std::shared_ptr<zivid_interfaces::srv::IsConnected::Response> response)
{
  response->is_connected = camera_status_ == CameraStatus::Connected;
}

void ZividCamNode::publishFrame(const Zivid::Frame & frame)
{
  RCLCPP_INFO_STREAM(get_logger(), "publishing frame ...");
  const bool publish_points_xyz = shouldPublishPointsXYZ();
  const bool publish_points_xyzrgba = shouldPublishPointsXYZRGBA();
  const bool publish_color_img = shouldPublishColorImg();
  const bool publish_depth_img = shouldPublishDepthImg();
  const bool publish_snr_img = shouldPublishSnrImg();
  const bool publish_normals_xyz = shouldPublishNormalsXYZ();

  if (
    publish_points_xyz || publish_points_xyzrgba || publish_color_img || publish_depth_img ||
    publish_snr_img || publish_normals_xyz) {
    const auto color_space = colorSpace();
    const auto header = makeHeader();
    auto point_cloud = frame.pointCloud();

    // The point cloud is transformed from millimeters to meters. This is a destructive
    // operation on the frame. We check if the transform has already been applied to
    // make this function safe to call on an already-processed frame.
    const auto& transform = point_cloud.transformationMatrix();
    if (std::equal(transform.begin(), transform.end(), Zivid::Matrix4x4::identity().begin()))
    {
      RCLCPP_INFO_STREAM(get_logger(), "Transforming point cloud from mm to m.");
      // Transform point cloud from millimeters (Zivid SDK) to meter (ROS).
      const float scale = 0.001f;
      const auto transformation_matrix =
        Zivid::Matrix4x4{scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, scale, 0, 0, 0, 0, 1};
      point_cloud.transform(transformation_matrix);
    }
    else {
      RCLCPP_INFO_STREAM(get_logger(), "Point cloud already transformed, skipping transform in publishFrame.");
    }
    if (publish_points_xyz) {
      publishPointCloudXYZ(header, point_cloud);
    }
    if (publish_points_xyzrgba) {
      publishPointCloudXYZRGBA(header, point_cloud, color_space);
    }
    if (publish_color_img || publish_depth_img || publish_snr_img) {
      auto intrinsics = [&] {
        const auto intrinsics_source = intrinsicsSource();
        switch (intrinsics_source) {
          case IntrinsicsSource::Camera:
            return Zivid::Experimental::Calibration::intrinsics(*camera_);
          case IntrinsicsSource::Frame:
            return Zivid::Experimental::Calibration::estimateIntrinsics(frame);
          default:
            throw std::runtime_error(
              "Internal error: Unknown intrinsics source value " +
              std::to_string(static_cast<int>(intrinsics_source)));
        }
      }();

      const auto camera_info =
        makeCameraInfo(header, point_cloud.width(), point_cloud.height(), intrinsics);

      if (publish_color_img) {
        publishColorImage(header, camera_info, point_cloud, color_space);
      }
      if (publish_depth_img) {
        publishDepthImage(header, camera_info, point_cloud);
      }
      if (publish_snr_img) {
        publishSnrImage(header, camera_info, point_cloud);
      }
    }
    if (publish_normals_xyz) {
      publishNormalsXYZ(header, point_cloud);
    }
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      __func__ << ": capture was called, but no subscribers active and 0 messages sent");
  }
}

void ZividCamNode::publishFrame2D(const Zivid::Frame2D & frame2D)
{
  if (shouldPublishColorImg()) {
    const auto color_space = colorSpace();
    const auto header = makeHeader();
    const auto intrinsics = Zivid::Experimental::Calibration::intrinsics(*camera_);

    switch (color_space) {
      case ColorSpace::sRGB: {
        auto image = frame2D.imageRGBA_SRGB();
        const auto camera_info = makeCameraInfo(header, image.width(), image.height(), intrinsics);
        publishColorImage(header, camera_info, image);
      } break;
      case ColorSpace::LinearRGB: {
        auto image = frame2D.imageRGBA();
        const auto camera_info = makeCameraInfo(header, image.width(), image.height(), intrinsics);
        publishColorImage(header, camera_info, image);
      } break;
      default:
        throw std::runtime_error(
          "Internal error: Unknown color space value " +
          std::to_string(static_cast<int>(color_space)));
    }
  }
}

bool ZividCamNode::shouldPublishPointsXYZRGBA() const
{
  return points_xyzrgba_publisher_->get_subscription_count() > 0 ||
         use_latched_publisher_for_points_xyzrgba_;
}

bool ZividCamNode::shouldPublishPointsXYZ() const
{
  return points_xyz_publisher_->get_subscription_count() > 0 ||
         use_latched_publisher_for_points_xyz_;
}

bool ZividCamNode::shouldPublishColorImg() const
{
  return color_image_publisher_.getNumSubscribers() > 0 || use_latched_publisher_for_color_image_;
}

bool ZividCamNode::shouldPublishDepthImg() const
{
  return depth_image_publisher_.getNumSubscribers() > 0 || use_latched_publisher_for_depth_image_;
}

bool ZividCamNode::shouldPublishSnrImg() const
{
  return snr_image_publisher_.getNumSubscribers() > 0 || use_latched_publisher_for_snr_image_;
}

bool ZividCamNode::shouldPublishNormalsXYZ() const
{
  return normals_xyz_publisher_->get_subscription_count() > 0 ||
         use_latched_publisher_for_normals_xyz_;
}

std_msgs::msg::Header ZividCamNode::makeHeader()
{
  std_msgs::msg::Header header;
  header.stamp = get_clock()->now();
  header.frame_id = frame_id_;
  return header;
}

void ZividCamNode::publishPointCloudXYZ(
  const std_msgs::msg::Header & header, const Zivid::PointCloud & point_cloud)
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing " << points_xyz_publisher_->get_topic_name());

  // We are using the Zivid::XYZW type here for compatibility with the pcl::PointXYZ type, which
  // contains a padding float for performance reasons. We could use the "pcl_conversion" utility
  // functions to construct the PointCloud2 message. However, those are observed to add significant
  // overhead due to extra unnecessary copies of the data.
  using ZividDataType = Zivid::PointXYZW;
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->fields.reserve(3);
  msg->fields.push_back(createPointField("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->is_dense = false;
  msg->point_step = sizeof(ZividDataType);
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step * msg->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType *>(msg->data.data()));
  points_xyz_publisher_->publish(std::move(msg));
}

void ZividCamNode::publishPointCloudXYZRGBA(
  const std_msgs::msg::Header & header, const Zivid::PointCloud & point_cloud,
  ColorSpace color_space)
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing " << points_xyzrgba_publisher_->get_topic_name());

  auto msg = [&] {
    switch (color_space) {
      case ColorSpace::sRGB:
        return makePointCloud2Msg<Zivid::PointXYZColorBGRA_SRGB>(point_cloud, header);
      case ColorSpace::LinearRGB:
        return makePointCloud2Msg<Zivid::PointXYZColorBGRA>(point_cloud, header);
      default:
        throw std::runtime_error(
          "Internal error: Unknown color space value " +
          std::to_string(static_cast<int>(color_space)));
    }
  }();
  points_xyzrgba_publisher_->publish(std::move(msg));
}

void ZividCamNode::publishColorImage(
  const std_msgs::msg::Header & header,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
  const Zivid::PointCloud & point_cloud, ColorSpace color_space)
{
  RCLCPP_INFO_STREAM(
    get_logger(), "Publishing " << color_image_publisher_.getTopic() << " from point cloud");
  sensor_msgs::msg::Image::SharedPtr image = [&] {
    switch (color_space) {
      case ColorSpace::sRGB:
        return makePointCloudImage<Zivid::ColorRGBA_SRGB>(
          point_cloud, header, sensor_msgs::image_encodings::RGBA8);
      case ColorSpace::LinearRGB:
        return makePointCloudImage<Zivid::ColorRGBA>(
          point_cloud, header, sensor_msgs::image_encodings::RGBA8);
      default:
        throw std::runtime_error(
          "Internal error: Unknown color space value " +
          std::to_string(static_cast<int>(color_space)));
    }
  }();
  color_image_publisher_.publish(image, camera_info);
}

void ZividCamNode::publishColorImage(
  const std_msgs::msg::Header & header,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
  const Zivid::Image<Zivid::ColorRGBA> & image)
{
  RCLCPP_INFO_STREAM(
    get_logger(), "Publishing " << color_image_publisher_.getTopic() << " from linear RGB image");
  auto msg =
    makeImageFromZividImage<Zivid::ColorRGBA>(image, header, sensor_msgs::image_encodings::RGBA8);
  color_image_publisher_.publish(msg, camera_info);
}

void ZividCamNode::publishColorImage(
  const std_msgs::msg::Header & header,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
  const Zivid::Image<Zivid::ColorRGBA_SRGB> & image)
{
  RCLCPP_INFO_STREAM(
    get_logger(), "Publishing " << color_image_publisher_.getTopic() << " from sRGB image");
  auto msg = makeImageFromZividImage<Zivid::ColorRGBA_SRGB>(
    image, header, sensor_msgs::image_encodings::RGBA8);
  color_image_publisher_.publish(msg, camera_info);
}

void ZividCamNode::publishDepthImage(
  const std_msgs::msg::Header & header,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
  const Zivid::PointCloud & point_cloud)
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing " << depth_image_publisher_.getTopic());
  auto image = makePointCloudImage<Zivid::PointZ>(
    point_cloud, header, sensor_msgs::image_encodings::TYPE_32FC1);
  depth_image_publisher_.publish(image, camera_info);
}

void ZividCamNode::publishSnrImage(
  const std_msgs::msg::Header & header,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
  const Zivid::PointCloud & point_cloud)
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing " << snr_image_publisher_.getTopic());
  auto image =
    makePointCloudImage<Zivid::SNR>(point_cloud, header, sensor_msgs::image_encodings::TYPE_32FC1);
  snr_image_publisher_.publish(image, camera_info);
}

void ZividCamNode::publishNormalsXYZ(
  const std_msgs::msg::Header & header, const Zivid::PointCloud & point_cloud)
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing " << normals_xyz_publisher_->get_topic_name());

  using ZividDataType = Zivid::NormalXYZ;
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  fillCommonMsgFields(*msg, header, point_cloud.width(), point_cloud.height());
  msg->fields.reserve(3);
  msg->fields.push_back(createPointField("normal_x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("normal_y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->fields.push_back(createPointField("normal_z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  msg->is_dense = false;
  msg->point_step = sizeof(ZividDataType);
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize(msg->row_step * msg->height);
  point_cloud.copyData<ZividDataType>(reinterpret_cast<ZividDataType *>(msg->data.data()));
  normals_xyz_publisher_->publish(std::move(msg));
}

sensor_msgs::msg::CameraInfo::ConstSharedPtr ZividCamNode::makeCameraInfo(
  const std_msgs::msg::Header & header, std::size_t width, std::size_t height,
  const Zivid::CameraIntrinsics & intrinsics)
{
  auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  msg->header = header;
  msg->width = static_cast<uint32_t>(width);
  msg->height = static_cast<uint32_t>(height);
  msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // k1, k2, t1, t2, k3
  const auto distortion = intrinsics.distortion();
  msg->d.resize(5);
  msg->d[0] = distortion.k1().value();
  msg->d[1] = distortion.k2().value();
  msg->d[2] = distortion.p1().value();
  msg->d[3] = distortion.p2().value();
  msg->d[4] = distortion.k3().value();

  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  const auto camera_matrix = intrinsics.cameraMatrix();
  msg->k[0] = camera_matrix.fx().value();
  msg->k[2] = camera_matrix.cx().value();
  msg->k[4] = camera_matrix.fy().value();
  msg->k[5] = camera_matrix.cy().value();
  msg->k[8] = 1;

  // R (identity)
  msg->r[0] = 1;
  msg->r[4] = 1;
  msg->r[8] = 1;

  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  msg->p[0] = camera_matrix.fx().value();
  msg->p[2] = camera_matrix.cx().value();
  msg->p[5] = camera_matrix.fy().value();
  msg->p[6] = camera_matrix.cy().value();
  msg->p[10] = 1;

  return msg;
}

Zivid::Frame ZividCamNode::invokeCaptureAndPublishFrame(const Zivid::Settings & settings)
{
  RCLCPP_INFO_STREAM(get_logger(), __func__);

  serviceHandlerHandleCameraConnectionLoss();

  RCLCPP_INFO(get_logger(), "Capturing with %zd acquisition(s)", settings.acquisitions().size());
  RCLCPP_DEBUG_STREAM(get_logger(), settings);
  const auto frame = camera_->capture(settings);
  publishFrame(frame);
  return frame;
}

void ZividCamNode::logErrorAndThrowRuntimeException(const std::string & message)
{
  zivid_camera::logErrorToLoggerAndThrowRuntimeException(get_logger(), message);
}

ColorSpace ZividCamNode::colorSpace() const
{
  const auto color_space_str = get_parameter(ParamNames::color_space).as_string();
  return parameterStringToEnum(
    ParamNames::color_space, color_space_str, color_space_name_value_map_);
}

IntrinsicsSource ZividCamNode::intrinsicsSource() const
{
  const auto intrinsics_source = get_parameter(ParamNames::intrinsics_source).as_string();
  return parameterStringToEnum(
    ParamNames::intrinsics_source, intrinsics_source, intrinsics_source_name_value_map_);
}

void ZividCamNode::exportFrame(
  const Zivid::Frame & frame, const std::string & file_name, ColorSpace color_space)
{
  namespace PointCloudExport = Zivid::Experimental::PointCloudExport;

  PointCloudExport::ColorSpace export_color_space = [&]() {
    switch (color_space) {
      case ColorSpace::LinearRGB:
        return PointCloudExport::ColorSpace::linearRGB;
      case ColorSpace::sRGB:
        return PointCloudExport::ColorSpace::sRGB;
      default:
        throw std::runtime_error("Enum `color_space` out of range.");
    }
  }();

  const auto extension = [&]() {
    const auto pos = file_name.find_last_of('.');
    return file_name.substr(pos + 1);
  }();

  if (extension == "zdf") {
    const auto spec = PointCloudExport::FileFormat::ZDF(file_name);
    PointCloudExport::exportFrame(frame, spec);
  } else if (extension == "ply") {
    const auto spec = PointCloudExport::FileFormat::PLY(
      file_name, PointCloudExport::FileFormat::PLY::Layout::ordered, export_color_space);
    PointCloudExport::exportFrame(frame, spec);
  } else if (extension == "pcd") {
    const auto spec = PointCloudExport::FileFormat::PCD(file_name, export_color_space);
    PointCloudExport::exportFrame(frame, spec);
  } else if (extension == "xyz") {
    const auto spec = PointCloudExport::FileFormat::XYZ(file_name, export_color_space);
    PointCloudExport::exportFrame(frame, spec);
  } else {
    throw std::runtime_error("Unknown extension `" + extension + "`.");
  }
}
}  // namespace zivid_camera_node

// #include "rclcpp_components/register_node_macro.hpp"

// #ifdef __clang__
// #pragma clang diagnostic push
// #pragma clang diagnostic ignored "-Wglobal-constructors"
// #pragma clang diagnostic ignored "-Wexit-time-destructors"
// #endif

// RCLCPP_COMPONENTS_REGISTER_NODE(zivid_camera_node::ZividCamNode)

// #ifdef __clang__
// #pragma clang diagnostic pop
// #endif