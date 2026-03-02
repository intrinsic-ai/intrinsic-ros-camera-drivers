#ifndef FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_ADAPTER_NODE_H_
#define FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_ADAPTER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "depthai_ros_driver/driver.hpp"
#include "flowstate_common/base_adapter_node.h"
#include "snapshot_interfaces/srv/describe.hpp"
#include "snapshot_interfaces/srv/snapshot.hpp"

namespace flowstate_luxonis {

class AdapterNode : public flowstate_common::BaseAdapterNode {
 public:
  AdapterNode(const std::string& serial,
              const std::vector<std::string>& locators);

 private:
  absl::Status Main() override;
  std::string ColorImageTopic() const override;
  absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
  BuildDescribeResponse(
      snapshot_interfaces::srv::Describe::Response& response) override;
  absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
  BuildSnapshotResponse(
      snapshot_interfaces::srv::Snapshot::Response& response) override;

  std::shared_ptr<depthai_ros_driver::Driver> luxonis_node_;
};

}  // namespace flowstate_luxonis

#endif  // FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_ADAPTER_NODE_H_
