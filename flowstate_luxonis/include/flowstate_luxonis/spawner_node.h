#ifndef FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_SPAWNER_NODE_H_
#define FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_SPAWNER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "flowstate_common/camera_spawner_node.h"
#include "flowstate_luxonis/adapter_node.h"
#include <XLink/XLinkPublicDefines.h>

namespace flowstate_luxonis {

/**
 * @class SpawnerNode
 * @brief Discovers and spawns adapter nodes for Luxonis (OAK) cameras.
 *
 * Inherits from the common CameraSpawnerNode base class and implements
 * Luxonis-specific camera discovery using the XLink API.
 */
class SpawnerNode : public flowstate_common::CameraSpawnerNode {
 public:
  SpawnerNode();

 private:
  void UpdateCameraList() override;
  bool IsAlreadySpawned(const std::string& serial) const override;
  std::string GetDiscoveredCameraSerial(size_t index) const override;

  std::string DeviceStateToString(XLinkDeviceState_t state);

  std::vector<std::unique_ptr<AdapterNode>> spawned_nodes_;
};

}  // namespace flowstate_luxonis

#endif  // FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_SPAWNER_NODE_H_
