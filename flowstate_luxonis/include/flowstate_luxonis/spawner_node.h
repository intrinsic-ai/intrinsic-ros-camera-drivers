#ifndef FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_SPAWNER_NODE_H_
#define FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_SPAWNER_NODE_H_

#include <XLink/XLinkPublicDefines.h>

#include <string>

#include "flowstate_common/base_spawner_node.h"
#include "flowstate_luxonis/adapter_node.h"

namespace flowstate_luxonis {

class SpawnerNode : public flowstate_common::BaseSpawnerNode {
 public:
  SpawnerNode();

 protected:
  std::vector<std::string> GetSerials() override;
  std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>> SpawnNodes(
      const std::vector<std::string>& serials) override;

 private:
  std::string DeviceStateToString(XLinkDeviceState_t state);
};

}  // namespace flowstate_luxonis

#endif  // FLOWSTATE_LUXONIS_FLOWSTATE_LUXONIS_SPAWNER_NODE_H_
