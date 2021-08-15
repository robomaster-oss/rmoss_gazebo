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
#include "rmoss_ign_base/ign_chassis_cmd.hpp"

#include <memory>
#include <string>

namespace rmoss_ign_base
{


IgnChassisCmd::IgnChassisCmd(
  const std::shared_ptr<ignition::transport::Node> & ign_node,
  const std::string & ign_chassis_cmd_topic)
{
  ign_node_ = ign_node;
  ign_chassis_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    ign_node_->Advertise<ignition::msgs::Twist>(ign_chassis_cmd_topic));
}

void IgnChassisCmd::publish(double v_x, double v_y, double v_w)
{
  ignition::msgs::Twist ign_msg;
  ign_msg.mutable_linear()->set_x(v_x);
  ign_msg.mutable_linear()->set_y(v_y);
  ign_msg.mutable_angular()->set_z(v_w);
  ign_chassis_cmd_pub_->Publish(ign_msg);
}

}  // namespace rmoss_ign_base
