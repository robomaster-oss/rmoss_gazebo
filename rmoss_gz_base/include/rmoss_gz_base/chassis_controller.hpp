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

#ifndef RMOSS_GZ_BASE__CHASSIS_CONTROLLER_HPP_
#define RMOSS_GZ_BASE__CHASSIS_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rmoss_gz_base/pid.hpp"
#include "rmoss_interfaces/msg/chassis_cmd.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"
#include "hardware_interface.hpp"

namespace rmoss_gz_base
{

class ChassisController
{
public:
  ChassisController(
    rclcpp::Node::SharedPtr node,
    Actuator<geometry_msgs::msg::Twist>::SharedPtr chassis_actuator,
    Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_encoder,
    const std::string & controller_name = "chassis_controller");
  ~ChassisController() {}

public:
  void set_chassis_pid(struct PidParam pid_param);
  void reset();

private:
  void chassis_cb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg);
  void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void update();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<rmoss_interfaces::msg::ChassisCmd>::SharedPtr ros_chassis_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ros_cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  // actuator and sensor
  Actuator<geometry_msgs::msg::Twist>::SharedPtr chassis_actuator_;
  Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_encoder_;
  // target data
  double target_vx_{0};
  double target_vy_{0};
  double target_w_{0};
  geometry_msgs::msg::Twist target_vel_;
  // pid and pid parameter
  double cur_yaw_{0};
  PidParam chassis_pid_param_;
  ignition::math::PID chassis_pid_;
  // flag
  bool update_pid_flag_{true};
  bool follow_mode_flag_{true};
};


}  // namespace rmoss_gz_base

#endif  // RMOSS_GZ_BASE__CHASSIS_CONTROLLER_HPP_
