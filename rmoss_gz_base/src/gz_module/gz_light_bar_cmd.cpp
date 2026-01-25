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

#include "gz/msgs/int32.pb.h"

namespace rmoss_gz_base
{

GzLightBarCmd::GzLightBarCmd(
  std::shared_ptr<gz::transport::Node> gz_node,
  const std::string & gz_cmd_topic)
: gz_node_(gz_node)
{
  gz_cmd_pub_ = std::make_unique<gz::transport::Node::Publisher>(
      gz_node_->Advertise<gz::msgs::Int32>(gz_cmd_topic));
}

void GzLightBarCmd::set_state(int state)
{
  gz::msgs::Int32 gz_msg;
  gz_msg.set_data(state);
  gz_cmd_pub_->Publish(gz_msg);
}

}  // namespace rmoss_gz_base
