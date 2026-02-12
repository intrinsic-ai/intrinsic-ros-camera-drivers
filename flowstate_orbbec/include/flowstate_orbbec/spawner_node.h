#ifndef FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_
#define FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "flowstate_common/camera_spawner_node.h"
#include "flowstate_orbbec/adapter_node.h"

namespace flowstate_orbbec {

class SpawnerNode : public flowstate_common::CameraSpawnerNode {
 public:
  SpawnerNode();

 protected:
  void UpdateCameras() override;
};

}  // namespace flowstate_orbbec

#endif  // FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_
