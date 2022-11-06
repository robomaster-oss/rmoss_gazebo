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
#include "rmoss_gz_base/gz_shoot_actuator.hpp"

#include <memory>
#include <string>

namespace rmoss_gz_base
{

IgnShootActuator::IgnShootActuator(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<ignition::transport::Node> gz_node,
  const std::string & robot_name,
  const std::string & shooter_name)
: node_(node), gz_node_(gz_node)
{
  // create ignition pub
  std::string gz_shoot_cmd_topic = "/" + robot_name + "/" + shooter_name + "/shoot";
  gz_shoot_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    gz_node_->Advertise<ignition::msgs::Int32>(gz_shoot_cmd_topic));
  std::string gz_set_vel_topic = "/" + robot_name + "/" + shooter_name + "/set_vel";
  gz_set_vel_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    gz_node_->Advertise<ignition::msgs::Double>(gz_set_vel_topic));
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
    ignition::msgs::Double gz_msg;
    gz_msg.set_data(projectile_vel_);
    gz_set_vel_pub_->Publish(gz_msg);
  }
  // publish shoot msg
  ignition::msgs::Int32 gz_msg;
  if (data.projectile_num > remain_num_) {
    gz_msg.set_data(remain_num_);
  } else {
    gz_msg.set_data(data.projectile_num);
  }
  gz_shoot_cmd_pub_->Publish(gz_msg);
}

}  // namespace rmoss_gz_base
