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
#include "rmoss_gz_base/gz_gimbal_actuator.hpp"

#include <memory>
#include <string>

namespace rmoss_gz_base
{

IgnGimbalActuator::IgnGimbalActuator(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<ignition::transport::Node> gz_node,
  const std::string & gz_pitch_topic,
  const std::string & gz_yaw_topic)
: node_(node), gz_node_(gz_node)
{
  gz_pitch_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    gz_node_->Advertise<ignition::msgs::Double>(gz_pitch_topic));
  gz_yaw_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    gz_node_->Advertise<ignition::msgs::Double>(gz_yaw_topic));
}

void IgnGimbalActuator::set(const rmoss_interfaces::msg::Gimbal & data)
{
  if (!enable_) {
    return;
  }
  ignition::msgs::Double gz_msg;
  gz_msg.set_data(data.pitch);
  gz_pitch_pub_->Publish(gz_msg);
  gz_msg.set_data(data.yaw);
  gz_yaw_pub_->Publish(gz_msg);
}


}  // namespace rmoss_gz_base
