// Copyright 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RMOSS_IGN_BASE__IGN_LIGHT_BAR_CMD_HPP_
#define RMOSS_IGN_BASE__IGN_LIGHT_BAR_CMD_HPP_

#include <memory>
#include <string>

#include "ignition/transport/Node.hh"

namespace rmoss_ign_base
{

class IgnLightBarCmd
{
public:
  IgnLightBarCmd(
    std::shared_ptr<ignition::transport::Node> ign_node,
    const std::string & ign_cmd_topic);
  ~IgnLightBarCmd() {}

public:
  void set_state(int state);

private:
  std::shared_ptr<ignition::transport::Node> ign_node_;
  std::unique_ptr<ignition::transport::Node::Publisher> ign_cmd_pub_;
};

}  // namespace rmoss_ign_base

#endif  // RMOSS_IGN_BASE__IGN_LIGHT_BAR_CMD_HPP_
