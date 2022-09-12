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

#ifndef RMOSS_GZ_BASE__SHOOTER_CONTROLLER_HPP_
#define RMOSS_GZ_BASE__SHOOTER_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "ignition/transport/Node.hh"
#include "rclcpp/rclcpp.hpp"
#include "rmoss_interfaces/msg/shoot_cmd.hpp"
#include "hardware_interface.hpp"

namespace rmoss_gz_base
{

class ShooterController
{
public:
  ShooterController(
    rclcpp::Node::SharedPtr node,
    Actuator<rmoss_interfaces::msg::ShootCmd>::SharedPtr shoot_actuator,
    const std::string & controller_name = "chassis_controller");
  ~ShooterController() {}

private:
  void shoot_cb(const rmoss_interfaces::msg::ShootCmd::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  // ros pub and sub
  rclcpp::Subscription<rmoss_interfaces::msg::ShootCmd>::SharedPtr ros_shoot_cmd_sub_;
  Actuator<rmoss_interfaces::msg::ShootCmd>::SharedPtr shoot_actuator_;
};
}  // namespace rmoss_gz_base
#endif  // RMOSS_GZ_BASE__SHOOTER_CONTROLLER_HPP_
