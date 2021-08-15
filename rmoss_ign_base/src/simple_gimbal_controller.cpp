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

#include "rmoss_ign_base/simple_gimbal_controller.hpp"

#include <memory>
#include <string>

namespace rmoss_ign_base
{

SimpleGimbalController::SimpleGimbalController(
  const rclcpp::Node::SharedPtr & nh,
  const std::string & ros_cmd_topic,
  const std::string & ign_pitch_cmd_topic,
  const std::string & ign_yaw_cmd_topic,
  bool position_control)
{
  // ROS and Ignition node
  nh_ = nh;
  ign_node_ = std::make_shared<ignition::transport::Node>();
  is_position_controller_ = position_control;
  // create ros pub and sub
  ros_gimbal_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::GimbalCmd>(
    ros_cmd_topic,
    10, std::bind(&SimpleGimbalController::gimbal_cb, this, std::placeholders::_1));
  // create ignition pub
  ign_pitch_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    ign_node_->Advertise<ignition::msgs::Double>(ign_pitch_cmd_topic));
  ign_yaw_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    ign_node_->Advertise<ignition::msgs::Double>(ign_yaw_cmd_topic));
}

void SimpleGimbalController::gimbal_cb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg)
{
  ignition::msgs::Double ign_msg;
  if (is_position_controller_) {
    ign_msg.set_data(msg->position.pitch);
    ign_pitch_pub_->Publish(ign_msg);
    ign_msg.set_data(msg->position.yaw);
    ign_yaw_pub_->Publish(ign_msg);
  } else {
    ign_msg.set_data(msg->velocity.pitch);
    ign_pitch_pub_->Publish(ign_msg);
    ign_msg.set_data(msg->velocity.yaw);
    ign_yaw_pub_->Publish(ign_msg);
  }
}


}  // namespace rmoss_ign_base
