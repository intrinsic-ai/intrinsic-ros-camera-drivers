#ifndef FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_ADAPTER_NODE_H_
#define FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_ADAPTER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "flowstate_common/camera_adapter_node.h"
#include "orbbec_camera/ob_camera_node_driver.h"
#include "orbbec_camera_msgs/srv/set_int32.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace flowstate_orbbec {

/**
 * @class AdapterNode
 * @brief Flowstate adapter for Orbbec cameras.
 *
 * Inherits from the common CameraAdapterNode base class and implements
 * Orbbec-specific initialization, parameter handling, and topic discovery.
 */
class AdapterNode : public flowstate_common::CameraAdapterNode {
 public:
  /**
   * @brief Constructor initializes the Orbbec adapter.
   * @param serial Camera serial number
   * @param ip_address Camera IP address (for network cameras)
   */
  AdapterNode(const std::string& serial, const std::string& ip_address);

 private:
  absl::Status Main() override;
  std::string ColorImageTopic() const override;
  bool BuildDescribeResponse(
      snapshot_interfaces::srv::Describe::Response& response) override;
  bool BuildSnapshotResponse(
      snapshot_interfaces::srv::Snapshot::Response& response) override;

  void InitializeParameters();
  void PreSetParametersCallback(std::vector<rclcpp::Parameter>& parameters);
  rcl_interfaces::msg::SetParametersResult SetParametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);
  void PostSetParametersCallback(const std::vector<rclcpp::Parameter>& parameters);

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
              RCLCPP_INFO_STREAM(this->get_logger(),
                                 client->get_service_name() << "(" << value
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

  std::string IrImageTopic() const;
  std::string DepthImageTopic() const;

  std::unique_ptr<orbbec_camera::OBCameraNodeDriver> orbbec_node_;

  // Additional camera info and images for IR and depth
  rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr
      pre_set_parameters_callback_handle_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      on_set_parameters_callback_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
      post_set_parameters_callback_handle_;

  mutable absl::Mutex ir_camera_info_mutex_;
  std::unique_ptr<sensor_msgs::msg::CameraInfo> ir_camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr ir_info_sub_;

  mutable absl::Mutex depth_camera_info_mutex_;
  std::unique_ptr<sensor_msgs::msg::CameraInfo> depth_camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;

  mutable absl::Mutex ir_image_mutex_;
  std::unique_ptr<sensor_msgs::msg::Image> ir_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ir_image_sub_;

  mutable absl::Mutex depth_image_mutex_;
  std::unique_ptr<sensor_msgs::msg::Image> depth_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;

  bool auto_white_balance_ = true;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_auto_exposure_client_;
  rclcpp::Client<orbbec_camera_msgs::srv::SetInt32>::SharedPtr
      set_exposure_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
      set_auto_white_balance_client_;
  rclcpp::Client<orbbec_camera_msgs::srv::SetInt32>::SharedPtr
      set_white_balance_client_;
  rclcpp::Client<orbbec_camera_msgs::srv::SetInt32>::SharedPtr set_gain_client_;
};

}  // namespace flowstate_orbbec

#endif  // FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_ADAPTER_NODE_H_
