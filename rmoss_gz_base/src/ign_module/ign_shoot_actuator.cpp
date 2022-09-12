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
#include "rmoss_ign_base/ign_shoot_actuator.hpp"

#include <memory>
#include <string>

namespace rmoss_ign_base
{

IgnShootActuator::IgnShootActuator(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<ignition::transport::Node> ign_node,
  const std::string & robot_name,
  const std::string & shooter_name)
: node_(node), ign_node_(ign_node)
{
  // create ignition pub
  std::string ign_shoot_cmd_topic = "/" + robot_name + "/" + shooter_name + "/shoot";
  ign_shoot_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    ign_node_->Advertise<ignition::msgs::Int32>(ign_shoot_cmd_topic));
  std::string ign_set_vel_topic = "/" + robot_name + "/" + shooter_name + "/set_vel";
  ign_set_vel_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    ign_node_->Advertise<ignition::msgs::Double>(ign_set_vel_topic));
}

void IgnShootActuator::set(const rmoss_interfaces::msg::ShootCmd & data)
{
  if (remain_num_ <= 0) {
    enable_ = false;
  }
  if (!enable_) {
    return;
  }
  // set velocity
  if (std::fabs(data.projectile_velocity - projectile_vel_) < 0.001) {
    projectile_vel_ = data.projectile_velocity;
    ignition::msgs::Double ign_msg;
    ign_msg.set_data(projectile_vel_);
    ign_set_vel_pub_->Publish(ign_msg);
  }
  // publish shoot msg
  ignition::msgs::Int32 ign_msg;
  if (data.projectile_num > remain_num_) {
    ign_msg.set_data(remain_num_);
  } else {
    ign_msg.set_data(data.projectile_num);
  }
  ign_shoot_cmd_pub_->Publish(ign_msg);
}

}  // namespace rmoss_ign_base
