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

#ifndef RMOSS_GZ_BASE__GZ_SHOOT_ACTUATOR_HPP_
#define RMOSS_GZ_BASE__GZ_SHOOT_ACTUATOR_HPP_

#include <memory>
#include <string>

#include "ignition/transport/Node.hh"
#include "rmoss_interfaces/msg/shoot_cmd.hpp"
#include "hardware_interface.hpp"

namespace rmoss_gz_base
{

class IgnShootActuator : public Actuator<rmoss_interfaces::msg::ShootCmd>
{
public:
  IgnShootActuator(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ignition::transport::Node> gz_node,
    const std::string & robot_name,
    const std::string & shooter_name);
  ~IgnShootActuator() {}

  void set(const rmoss_interfaces::msg::ShootCmd & data) override;
  void enable(bool enable) {enable_ = enable;}
  void update_remain_num(int num) {remain_num_ = num;}

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  // ign pub and sub
  std::unique_ptr<ignition::transport::Node::Publisher> gz_shoot_cmd_pub_;
  std::unique_ptr<ignition::transport::Node::Publisher> gz_set_vel_pub_;
  // data
  double projectile_vel_{0};
  int remain_num_{200};
  bool enable_{false};
};

}  // namespace rmoss_gz_base

#endif  // RMOSS_GZ_BASE__GZ_SHOOT_ACTUATOR_HPP_
