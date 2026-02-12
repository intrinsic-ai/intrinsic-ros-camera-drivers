#ifndef FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_SPAWNER_NODE_H_
#define FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_SPAWNER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "flowstate_common/camera_spawner_node.h"
#include "flowstate_luxonis/adapter_node.h"
#include <XLink/XLinkPublicDefines.h>

namespace flowstate_luxonis {

class SpawnerNode : public flowstate_common::CameraSpawnerNode {
 public:
  SpawnerNode();

 protected:
  void UpdateCameras() override;

 private:
  std::string DeviceStateToString(XLinkDeviceState_t state);
};

}  // namespace flowstate_luxonis

#endif  // FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_SPAWNER_NODE_H_