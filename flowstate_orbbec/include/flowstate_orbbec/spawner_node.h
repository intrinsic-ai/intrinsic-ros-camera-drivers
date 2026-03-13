#ifndef FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_
#define FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "flowstate_common/base_spawner_node.h"
#include "flowstate_orbbec/adapter_node.h"

namespace flowstate_orbbec {

class SpawnerNode : public flowstate_common::BaseSpawnerNode {
 public:
  SpawnerNode();

 protected:
  std::vector<std::string> GetSerials() override;
  std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>> SpawnNodes(
      const std::vector<std::string>& serials) override;
};

}  // namespace flowstate_orbbec

#endif  // FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_
