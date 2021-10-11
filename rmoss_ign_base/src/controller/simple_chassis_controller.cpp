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

#include "rmoss_ign_base/simple_chassis_controller.hpp"

#include <memory>
#include <string>


namespace rmoss_ign_base
{


SimpleChassisController::SimpleChassisController(
  const rclcpp::Node::SharedPtr & nh,
  const std::string & ros_cmd_topic,
  const std::string & ign_cmd_topic)
{
  // ROS and Ignition node
  nh_ = nh;
  ign_node_ = std::make_shared<ignition::transport::Node>();
  // create ros pub and sub
  ros_chassis_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::ChassisCmd>(
    ros_cmd_topic,
    10, std::bind(&SimpleChassisController::chassis_cb, this, std::placeholders::_1));
  // create ignition pub
  ign_chassis_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    ign_node_->Advertise<ignition::msgs::Twist>(ign_cmd_topic));
}

void SimpleChassisController::chassis_cb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg)
{
  ignition::msgs::Twist ign_msg;
  ign_msg.mutable_linear()->set_x(msg->twist.linear.x);
  ign_msg.mutable_linear()->set_y(msg->twist.linear.y);
  ign_msg.mutable_angular()->set_z(msg->twist.angular.z);
  ign_chassis_cmd_pub_->Publish(ign_msg);
}


}  // namespace rmoss_ign_base
