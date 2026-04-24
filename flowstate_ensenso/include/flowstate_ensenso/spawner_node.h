#ifndef FLOWSTATE_ENSENSO_FLOWSTATE_ENSENSO_SPAWNER_NODE_H_
#define FLOWSTATE_ENSENSO_FLOWSTATE_ENSENSO_SPAWNER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "flowstate_common/base_spawner_node.h"

namespace flowstate_ensenso {

class SpawnerNode : public flowstate_common::BaseSpawnerNode {
 public:
  SpawnerNode();
  virtual ~SpawnerNode();

 protected:
  std::vector<std::string> GetSerials() override;
  std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>>
  SpawnNodes(const std::vector<std::string>& serials) override;
};

}  // namespace flowstate_ensenso

#endif  // FLOWSTATE_ENSENSO_FLOWSTATE_ENSENSO_SPAWNER_NODE_H_
