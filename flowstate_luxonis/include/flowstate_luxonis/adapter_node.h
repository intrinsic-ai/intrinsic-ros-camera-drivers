#ifndef FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_ADAPTER_NODE_H_
#define FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_ADAPTER_NODE_H_

#include <memory>
#include <string>

#include "absl/status/status.h"
#include "depthai_ros_driver/driver.hpp"
#include "flowstate_common/camera_adapter_node.h"

namespace flowstate_luxonis {

/**
 * @class AdapterNode
 * @brief Flowstate adapter for Luxonis (OAK) cameras.
 *
 * Inherits from the common CameraAdapterNode base class and implements
 * Luxonis-specific initialization and topic discovery.
 */
class AdapterNode : public flowstate_common::CameraAdapterNode {
 public:
  /**
   * @brief Constructor initializes the Luxonis adapter.
   * @param serial Camera serial number/MXID
   * @param ip_address Camera IP address (for network cameras)
   */
  AdapterNode(const std::string& serial, const std::string& ip_address);

 private:
  absl::Status Main() override;
  std::string ColorImageTopic() const override;

  std::shared_ptr<depthai_ros_driver::Driver> luxonis_node_;
};

}  // namespace flowstate_luxonis

#endif  // FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_ADAPTER_NODE_H_
