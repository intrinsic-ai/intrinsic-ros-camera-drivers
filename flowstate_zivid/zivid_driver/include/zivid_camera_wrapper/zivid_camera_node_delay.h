// Copyright 2024 Zivid AS
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Zivid AS nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <zivid_camera/visibility.hpp>
#include <zivid_interfaces/srv/camera_info_model_name.hpp>
#include <zivid_interfaces/srv/camera_info_serial_number.hpp>
#include <zivid_interfaces/srv/capture_and_save.hpp>
#include <zivid_interfaces/srv/capture_assistant_suggest_settings.hpp>
#include <zivid_interfaces/srv/is_connected.hpp>

#include "zivid_camera/capture_settings_controller.hpp"

#include "snapshot_interfaces/srv/describe.hpp"
#include "snapshot_interfaces/msg/sensor_info.hpp"
#include "snapshot_interfaces/srv/snapshot.hpp"
#include <Zivid/Application.h>
#include <Zivid/Camera.h>
#include <Zivid/CameraIntrinsics.h>
#include <Zivid/Color.h>
#include <Zivid/Frame.h>
#include <Zivid/Frame2D.h>
#include <Zivid/Image.h>
#include <Zivid/PointCloud.h>
#include <Zivid/Settings.h>
#include <Zivid/Settings2D.h>

#include <atomic>
#include <mutex>

namespace Zivid
{
class Application;
class Camera;
class CameraIntrinsics;
struct ColorRGBA;
struct ColorSRGB;
using ColorRGBA_SRGB = ColorSRGB;
class Frame;
class Frame2D;
template <typename T>
class Image;
class PointCloud;
class Settings2D;
class Settings;
}  // namespace Zivid

namespace zivid_camera_node
{
enum class CameraStatus
{
  Idle,
  Connected,
  Disconnected
};
enum class ColorSpace
{
  sRGB,
  LinearRGB,
};
enum class IntrinsicsSource
{
  Camera,
  Frame,
};

struct ZividCaptureParameters
{
  double acquisition_frame_rate;
  double exposure_time;
  double gain;
  double gamma;
  bool hdr_enabled;
  double hdr_bracket_0;
  double hdr_bracket_1;
  double hdr_bracket_2;
  double projector_brightness;
  double aperture;
  bool outlier_removal_enabled;
  double outlier_removal_threshold;
  std::string trigger_selector;

  static ZividCaptureParameters boot_defaults()
  {
    // Default values are based on the settings from the sample files, with other common settings set to their default values.
    return { 50.0,      // acquisition_frame_rate
             0.008333,  // exposure_time (8333us)
             1.0,       // gain
             1.0,       // gamma
             false,     // hdr_enabled
             0.008,     // hdr_bracket_0 (8000 us)
             0.016,     // hdr_bracket_1 (16000 us)
             0.032,     // hdr_bracket_2 (32000 us)
             1.0,       // projector_brightness
             5.66,      // aperture
             true,      // outlier_removal_enabled
             5.0,       // outlier_removal_threshold
             "Software" };  // trigger_selector
  }
};


class ZividCamNode : public rclcpp::Node
{
public:
  // This constructor is for standalone use and is not needed by the wrapper.
  // ZIVID_CAMERA_ROS_PUBLIC ZividCamNode(const std::string & node_name, const rclcpp::NodeOptions & options, Zivid::Application & zivid_app);
  ZIVID_CAMERA_ROS_PUBLIC ZividCamNode(const std::string & node_name, const rclcpp::NodeOptions & options, Zivid::Camera camera, Zivid::Application & zivid_app);
  ~ZividCamNode() override;
  ZIVID_CAMERA_ROS_PUBLIC Zivid::Application & zividApplication();

private:
  void init();
  Zivid::Settings generateZividSettings() const;
  void onCameraConnectionKeepAliveTimeout();
  void reconnectToCameraIfNecessary();
  void setCameraStatus(CameraStatus camera_status);
  rcl_interfaces::msg::SetParametersResult setParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  void cameraInfoModelNameServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Request> request,
    std::shared_ptr<zivid_interfaces::srv::CameraInfoModelName::Response> response);
  void cameraInfoSerialNumberServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Request> request,
    std::shared_ptr<zivid_interfaces::srv::CameraInfoSerialNumber::Response> response);
  void captureServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<snapshot_interfaces::srv::Snapshot::Request> request,
    std::shared_ptr<snapshot_interfaces::srv::Snapshot::Response> response);
  void populateSnapshotResponse(
    std::shared_ptr<snapshot_interfaces::srv::Snapshot::Response> response,
    const Zivid::Frame & frame);
  void captureAndSaveServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::CaptureAndSave::Request> request,
    std::shared_ptr<zivid_interfaces::srv::CaptureAndSave::Response> response);
  void capture2DServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void captureAssistantSuggestSettingsServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::CaptureAssistantSuggestSettings::Request> request,
    std::shared_ptr<zivid_interfaces::srv::CaptureAssistantSuggestSettings::Response> response);
  void serviceHandlerHandleCameraConnectionLoss();
  void isConnectedServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::IsConnected::Request> request,
    std::shared_ptr<zivid_interfaces::srv::IsConnected::Response> response);
  // Describe service handler
    void describe_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<snapshot_interfaces::srv::Describe::Request>
          request,
      const std::shared_ptr<snapshot_interfaces::srv::Describe::Response>
          response);
  void connectServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void updateSettingsYamlCallback();

  
    void publishFrame(const Zivid::Frame & frame);
  void publishFrame2D(const Zivid::Frame2D & frame2D);
  Zivid::Frame invokeCaptureAndPublishFrame(const Zivid::Settings & settings);
  bool shouldPublishPointsXYZ() const;
  bool shouldPublishPointsXYZRGBA() const;
  bool shouldPublishColorImg() const;
  bool shouldPublishDepthImg() const;
  bool shouldPublishSnrImg() const;
  bool shouldPublishNormalsXYZ() const;
  std_msgs::msg::Header makeHeader();
  void publishPointCloudXYZ(
    const std_msgs::msg::Header & header, const Zivid::PointCloud & point_cloud);
  void publishPointCloudXYZRGBA(
    const std_msgs::msg::Header & header, const Zivid::PointCloud & point_cloud,
    ColorSpace color_space);
  void publishColorImage(
    const std_msgs::msg::Header & header,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
    const Zivid::PointCloud & point_cloud, ColorSpace color_space);
  void publishColorImage(
    const std_msgs::msg::Header & header,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
    const Zivid::Image<Zivid::ColorRGBA> & image);
  void publishColorImage(
    const std_msgs::msg::Header & header,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
    const Zivid::Image<Zivid::ColorRGBA_SRGB> & image);
  void publishDepthImage(
    const std_msgs::msg::Header & header,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
    const Zivid::PointCloud & point_cloud);
  void publishSnrImage(
    const std_msgs::msg::Header & header,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info,
    const Zivid::PointCloud & point_cloud);
  void publishNormalsXYZ(
    const std_msgs::msg::Header & header, const Zivid::PointCloud & point_cloud);
  sensor_msgs::msg::CameraInfo::ConstSharedPtr makeCameraInfo(
    const std_msgs::msg::Header & header, std::size_t width, std::size_t height,
    const Zivid::CameraIntrinsics & intrinsics);
  [[noreturn]] void logErrorAndThrowRuntimeException(const std::string & message);
  ColorSpace colorSpace() const;
  IntrinsicsSource intrinsicsSource() const;
  static void exportFrame(
    const Zivid::Frame & frame, const std::string & file_name, ColorSpace color_space);

  friend class ControllerInterface;

  std::map<std::string, ColorSpace> color_space_name_value_map_;
  std::map<std::string, IntrinsicsSource> intrinsics_source_name_value_map_;
  rclcpp::TimerBase::SharedPtr camera_connection_keepalive_timer_;
  bool use_latched_publisher_for_points_xyz_{false};
  bool use_latched_publisher_for_points_xyzrgba_{false};
  bool use_latched_publisher_for_color_image_{false};
  bool use_latched_publisher_for_depth_image_{false};
  bool use_latched_publisher_for_snr_image_{false};
  bool use_latched_publisher_for_normals_xyz_{false};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_xyz_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_xyzrgba_publisher_;
  image_transport::CameraPublisher color_image_publisher_;
  image_transport::CameraPublisher depth_image_publisher_;
  image_transport::CameraPublisher snr_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr normals_xyz_publisher_;
  rclcpp::Service<zivid_interfaces::srv::CameraInfoSerialNumber>::SharedPtr
    camera_info_serial_number_service_;
  rclcpp::Service<zivid_interfaces::srv::CameraInfoModelName>::SharedPtr
    camera_info_model_name_service_;
  rclcpp::Service<snapshot_interfaces::srv::Snapshot>::SharedPtr capture_service_;
  rclcpp::Service<zivid_interfaces::srv::CaptureAndSave>::SharedPtr capture_and_save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_2d_service_;
  rclcpp::Service<zivid_interfaces::srv::CaptureAssistantSuggestSettings>::SharedPtr
    capture_assistant_suggest_settings_service_;
  rclcpp::Service<zivid_interfaces::srv::IsConnected>::SharedPtr is_connected_service_;
  rclcpp::Service<snapshot_interfaces::srv::Describe>::SharedPtr describe_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;

//   std::unique_ptr<DetectorController> detector_controller_;
//   std::unique_ptr<InfieldCorrectionController> infield_correction_controller_;
//   std::unique_ptr<HandEyeCalibrationController> hand_eye_calibration_controller_;
//   std::unique_ptr<ProjectionController> projection_controller_;

//   std::unique_ptr<Zivid::Application> zivid_;
  Zivid::Application & zivid_app_ref_;
  CameraStatus camera_status_{CameraStatus::Idle};
  std::unique_ptr<zivid_camera::CaptureSettingsController<Zivid::Settings>> settings_controller_;
  std::unique_ptr<zivid_camera::CaptureSettingsController<Zivid::Settings2D>> settings_2d_controller_;
  // The callback must be declared after the settings controllers since the callback references
  // both controllers. Otherwise, the callback could run before the controllers are initialized,
  // which is undefined behavior.
  OnSetParametersCallbackHandle::SharedPtr set_parameters_callback_handle_;

  rclcpp::TimerBase::SharedPtr update_settings_yaml_timer_;
  std::atomic<bool> individual_settings_dirty_{false};
  mutable std::mutex capture_params_mutex_;
  std::unique_ptr<Zivid::Camera> camera_;
  ZividCaptureParameters capture_params_;
  std::string frame_id_;
  std::chrono::milliseconds connection_timeout_{30000};
};
}  // namespace zivid_camera_node
