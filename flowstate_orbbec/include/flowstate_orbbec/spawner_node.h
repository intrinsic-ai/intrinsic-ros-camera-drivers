#ifndef FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_
#define FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "flowstate_common/camera_spawner_node.h"
#include "flowstate_orbbec/adapter_node.h"

namespace flowstate_orbbec {

/**
 * @class SpawnerNode
 * @brief Discovers and spawns adapter nodes for Orbbec cameras.
 *
 * Inherits from the common CameraSpawnerNode base class and implements
 * Orbbec-specific camera discovery using the OrbbecSDK.
 */
class SpawnerNode : public flowstate_common::CameraSpawnerNode {
 public:
  SpawnerNode();

 private:
  void UpdateCameraList() override;
  bool IsAlreadySpawned(const std::string& serial) const override;
  std::string GetDiscoveredCameraSerial(size_t index) const override;
  std::string GetDiscoveredCameraIp(size_t index) const override;

  std::vector<std::unique_ptr<AdapterNode>> spawned_nodes_;
};

}  // namespace flowstate_orbbec

#endif  // FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_
