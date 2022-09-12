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
#include "rmoss_ign_base/ign_light_bar_cmd.hpp"

#include <memory>
#include <string>

namespace rmoss_ign_base
{


IgnLightBarCmd::IgnLightBarCmd(
  std::shared_ptr<ignition::transport::Node> ign_node,
  const std::string & ign_cmd_topic)
: ign_node_(ign_node)
{
  ign_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    ign_node_->Advertise<ignition::msgs::Int32>(ign_cmd_topic));
}

void IgnLightBarCmd::set_state(int state)
{
  ignition::msgs::Int32 ign_msg;
  ign_msg.set_data(state);
  ign_cmd_pub_->Publish(ign_msg);
}

}  // namespace rmoss_ign_base
