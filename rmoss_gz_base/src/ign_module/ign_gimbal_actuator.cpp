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
#include "rmoss_ign_base/ign_gimbal_actuator.hpp"

#include <memory>
#include <string>

namespace rmoss_ign_base
{

IgnGimbalActuator::IgnGimbalActuator(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<ignition::transport::Node> ign_node,
  const std::string & ign_pitch_topic,
  const std::string & ign_yaw_topic)
: node_(node), ign_node_(ign_node)
{
  ign_pitch_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    ign_node_->Advertise<ignition::msgs::Double>(ign_pitch_topic));
  ign_yaw_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    ign_node_->Advertise<ignition::msgs::Double>(ign_yaw_topic));
}

void IgnGimbalActuator::set(const rmoss_interfaces::msg::Gimbal & data)
{
  if (!enable_) {
    return;
  }
  ignition::msgs::Double ign_msg;
  ign_msg.set_data(data.pitch);
  ign_pitch_pub_->Publish(ign_msg);
  ign_msg.set_data(data.yaw);
  ign_yaw_pub_->Publish(ign_msg);
}


}  // namespace rmoss_ign_base
