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

#ifndef RMOSS_GZ_BASE__GIMBAL_CONTROLLER_HPP_
#define RMOSS_GZ_BASE__GIMBAL_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rmoss_interfaces/msg/gimbal_cmd.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"
#include "pid.hpp"
#include "hardware_interface.hpp"

namespace rmoss_gz_base
{

class GimbalController
{
public:
  GimbalController(
    rclcpp::Node::SharedPtr node,
    Actuator<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_vel_actuator,
    Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_pos_sensor,
    const std::string & controller_name = "gimbal_controller");
  ~GimbalController() {}

public:
  void set_yaw_pid(struct PidParam pid_param);
  void set_pitch_pid(struct PidParam pid_param);
  // set gimbal's motor limit (TODO)
  // void set_yaw_motor_limit(double min, double max) {}
  // void set_pitch_motor_limit(double min, double max) {}
  void reset();

private:
  void gimbal_cb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg);
  void update();
  void gimbal_state_timer_cb();

private:
  rclcpp::Node::SharedPtr node_;
  // ros pub and sub
  rclcpp::Subscription<rmoss_interfaces::msg::GimbalCmd>::SharedPtr ros_gimbal_cmd_sub_;
  rclcpp::Publisher<rmoss_interfaces::msg::Gimbal>::SharedPtr ros_gimbal_state_pub_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::TimerBase::SharedPtr gimbal_state_timer_;
  // control interface
  Actuator<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_vel_actuator_;
  Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_pos_sensor_;
  // target data
  double target_pitch_{0};
  double target_yaw_{0};
  double cur_pitch_{0};
  double cur_yaw_{0};
  // pid and pid parameter
  PidParam picth_pid_param_;
  PidParam yaw_pid_param_;
  ignition::math::PID picth_pid_;
  ignition::math::PID yaw_pid_;
  std::chrono::nanoseconds pid_period_;
  // flag
  bool update_pid_flag_{true};
};

}  // namespace rmoss_gz_base

#endif  // RMOSS_GZ_BASE__GIMBAL_CONTROLLER_HPP_
