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

#include "rmoss_gz_base/shooter_controller.hpp"

#include <memory>
#include <string>

namespace rmoss_gz_base
{

ShooterController::ShooterController(
  rclcpp::Node::SharedPtr node,
  Actuator<rmoss_interfaces::msg::ShootCmd>::SharedPtr shoot_actuator,
  const std::string & controller_name)
: node_(node), shoot_actuator_(shoot_actuator)
{
  (void)controller_name;
  // create ros pub and sub
  using namespace std::placeholders;
  std::string ros_shoot_cmd_topic = "robot_base/shoot_cmd";
  ros_shoot_cmd_sub_ = node_->create_subscription<rmoss_interfaces::msg::ShootCmd>(
    ros_shoot_cmd_topic, 10, std::bind(&ShooterController::shoot_cb, this, _1));
}

void ShooterController::shoot_cb(const rmoss_interfaces::msg::ShootCmd::SharedPtr msg)
{
  shoot_actuator_->set(*msg);
}

}  // namespace rmoss_gz_base
