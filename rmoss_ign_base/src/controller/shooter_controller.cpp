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

#include "rmoss_ign_base/shooter_controller.hpp"

#include <memory>
#include <string>

namespace rmoss_ign_base
{

ShooterController::ShooterController(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<ignition::transport::Node> ign_node,
  const std::string & ign_cmd_topic,
  const std::string & shooter_name)
{
  // ROS and Ignition node
  node_ = node;
  ign_node_ = std::make_shared<ignition::transport::Node>();
  // create ros pub and sub
  using namespace std::placeholders;
  std::string ros_shoot_cmd_topic = "robot_base/shoot_cmd";
  if (shooter_name != "") {
    ros_shoot_cmd_topic = "robot_base/" + shooter_name + "/shoot_cmd";
  }
  ros_shoot_cmd_sub_ = node_->create_subscription<rmoss_interfaces::msg::ShootCmd>(
    ros_shoot_cmd_topic, 10, std::bind(&ShooterController::shoot_cb, this, _1));
  // create ignition pub
  ign_shoot_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    ign_node_->Advertise<ignition::msgs::Int32>(ign_cmd_topic));
}

void ShooterController::shoot_cb(const rmoss_interfaces::msg::ShootCmd::SharedPtr msg)
{
  ignition::msgs::Int32 ign_msg;
  ign_msg.set_data(msg->projectile_num);
  ign_shoot_cmd_pub_->Publish(ign_msg);
  // std::cout << " shoot_cb,shoot num:" << (int)(msg->projectile_num) << std::endl;
}


}  // namespace rmoss_ign_base
