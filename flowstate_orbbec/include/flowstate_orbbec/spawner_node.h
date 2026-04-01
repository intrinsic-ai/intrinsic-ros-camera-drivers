#ifndef FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_
#define FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "flowstate_common/base_spawner_node.h"
#include "flowstate_orbbec/adapter_node.h"

namespace flowstate_orbbec {

class SpawnerNode : public flowstate_common::BaseSpawnerNode {
 public:
  SpawnerNode();

  // Apparently there are some times when it is forbidden to discover new
  // cameras, such as while a camera is rebooting. This appears to lead to
  // crashes in the process. We can work around this by using a static mutex
  // on the discovery calls, to ensure we are not rebooting a camera while
  // discovering, because this spawner is a singleton object.
  static absl::Mutex s_discovery_mutex;

 protected:
  std::vector<std::string> GetSerials() override;
  std::vector<std::shared_ptr<flowstate_common::BaseAdapterNode>> SpawnNodes(
      const std::vector<std::string>& serials) override;
};

}  // namespace flowstate_orbbec

#endif  // FLOWSTATE_ORBBEC_FLOWSTATE_ORBBEC_SPAWNER_NODE_H_
