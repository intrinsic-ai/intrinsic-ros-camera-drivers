#ifndef FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_ADAPTER_NODE_H_
#define FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_ADAPTER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flowstate_common/base_adapter_node.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "orbbec_camera/ob_camera_node_driver.h"
#include "orbbec_camera_msgs/srv/set_int32.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "snapshot_interfaces/srv/describe.hpp"
#include "snapshot_interfaces/srv/snapshot.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace flowstate_orbbec {

class AdapterNode : public flowstate_common::BaseAdapterNode {
 public:
  AdapterNode(const std::string& serial,
              const std::vector<std::string>& locators);
  virtual ~AdapterNode();

 private:
  absl::Status Main() override ABSL_LOCKS_EXCLUDED(timeout_mutex_);
  std::string ColorImageTopic() const override;
  absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
  BuildDescribeResponse() override;
  absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
  BuildSnapshotResponse() override;

  std::string LeftIrImageTopic() const;
  std::string RightIrImageTopic() const;
  std::string DepthImageTopic() const;

  void InitializeParameters();
  void CreateOrbbecNode();
  void DestroyOrbbecNode();
  rclcpp::NodeOptions CreateOrbbecNodeOptions(const std::string& serial);

  bool IsRgbEnabled() const;
  bool IsDepthEnabled() const;
  bool IsLeftIrEnabled() const;
  bool IsRightIrEnabled() const;

  rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr
      pre_set_parameters_callback_handle_;
  void PreSetParametersCallback(std::vector<rclcpp::Parameter>& parameters);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      on_set_parameters_callback_handle_;
  rcl_interfaces::msg::SetParametersResult SetParametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);

  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
      post_set_parameters_callback_handle_;
  void PostSetParametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);

  template <typename ServiceType, typename ValueType>
  void CallAsyncSet(std::shared_ptr<rclcpp::Client<ServiceType>> client,
                    const ValueType value, ValueType* shadow = nullptr) {
    auto request = std::make_shared<typename ServiceType::Request>();
    request->data = value;
    client->async_send_request(
        request, [this, client, value,
                  shadow](rclcpp::Client<ServiceType>::SharedFuture future) {
          if (future.valid()) {
            auto response = future.get();
            if (response->success) {
              RCLCPP_INFO_STREAM(this->get_logger(), client->get_service_name()
                                                         << "(" << value
                                                         << ") succeeded");
              if (shadow != nullptr) {
                *shadow = value;
              }
            } else {
              RCLCPP_ERROR_STREAM(this->get_logger(),
                                  client->get_service_name()
                                      << "(" << value << ") failed: "
                                      << response->message.c_str());
            }
          } else {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                client->get_service_name()
                                    << "(" << value
                                    << ") did not return a valid future");
          }
        });
  }

  absl::Status PopulateExtrinsicsIfNeeded();

  std::string OrbbecNodeNamespace();

  bool auto_white_balance_ = true;  // this resets WB if you disable it "again"
  std::unique_ptr<orbbec_camera::OBCameraNodeDriver> orbbec_node_;

  // IR Data
  std::unique_ptr<sensor_msgs::msg::CameraInfo> left_ir_camera_info_
      ABSL_GUARDED_BY(camera_info_mutex_);
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      left_ir_info_sub_;
  std::unique_ptr<sensor_msgs::msg::Image> left_ir_image_
      ABSL_GUARDED_BY(image_mutex_);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_ir_image_sub_;

  std::unique_ptr<sensor_msgs::msg::CameraInfo> right_ir_camera_info_
      ABSL_GUARDED_BY(camera_info_mutex_);
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      right_ir_info_sub_;
  std::unique_ptr<sensor_msgs::msg::Image> right_ir_image_
      ABSL_GUARDED_BY(image_mutex_);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_ir_image_sub_;

  // Depth Data
  std::unique_ptr<sensor_msgs::msg::CameraInfo> depth_camera_info_
      ABSL_GUARDED_BY(camera_info_mutex_);
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  std::unique_ptr<sensor_msgs::msg::Image> depth_image_
      ABSL_GUARDED_BY(image_mutex_);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_auto_exposure_client_;
  rclcpp::Client<orbbec_camera_msgs::srv::SetInt32>::SharedPtr
      set_exposure_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
      set_auto_white_balance_client_;
  rclcpp::Client<orbbec_camera_msgs::srv::SetInt32>::SharedPtr
      set_white_balance_client_;
  rclcpp::Client<orbbec_camera_msgs::srv::SetInt32>::SharedPtr set_gain_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr toggle_color_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr toggle_depth_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr toggle_left_ir_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr toggle_right_ir_client_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<geometry_msgs::msg::TransformStamped> color_transform_;
  std::unique_ptr<geometry_msgs::msg::TransformStamped> left_ir_transform_;
  std::unique_ptr<geometry_msgs::msg::TransformStamped> right_ir_transform_;

  rclcpp::Time t_last_right_ir_image_ ABSL_GUARDED_BY(timeout_mutex_);
  rclcpp::Time t_last_left_ir_image_ ABSL_GUARDED_BY(timeout_mutex_);
  rclcpp::Time t_last_depth_image_ ABSL_GUARDED_BY(timeout_mutex_);

  int reboot_count_ = 0;
  double fps_ = 5.0;

  std::unique_ptr<std::thread> orbbec_thread_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> orbbec_executor_;
};

}  // namespace flowstate_orbbec

#endif  // FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_ADAPTER_NODE_H_
