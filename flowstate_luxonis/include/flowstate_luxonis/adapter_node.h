/*
 * Copyright 2026 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
  absl::Status Main() override ABSL_LOCKS_EXCLUDED(timeout_mutex_);
  std::string ColorImageTopic() const override;
  absl::StatusOr<snapshot_interfaces::srv::Describe::Response>
    BuildDescribeResponse() override;
  absl::StatusOr<snapshot_interfaces::srv::Snapshot::Response>
    BuildSnapshotResponse() override;

  std::shared_ptr<depthai_ros_driver::Driver> luxonis_node_;
};

}  // namespace flowstate_luxonis

#endif  // FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_ADAPTER_NODE_H_
