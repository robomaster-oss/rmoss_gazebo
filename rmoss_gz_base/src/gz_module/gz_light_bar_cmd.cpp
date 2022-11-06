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
#include "rmoss_gz_base/gz_light_bar_cmd.hpp"

#include <memory>
#include <string>

namespace rmoss_gz_base
{


IgnLightBarCmd::IgnLightBarCmd(
  std::shared_ptr<ignition::transport::Node> gz_node,
  const std::string & gz_cmd_topic)
: gz_node_(gz_node)
{
  gz_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    gz_node_->Advertise<ignition::msgs::Int32>(gz_cmd_topic));
}

void IgnLightBarCmd::set_state(int state)
{
  ignition::msgs::Int32 gz_msg;
  gz_msg.set_data(state);
  gz_cmd_pub_->Publish(gz_msg);
}

}  // namespace rmoss_gz_base
