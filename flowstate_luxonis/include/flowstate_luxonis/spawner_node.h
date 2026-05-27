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
