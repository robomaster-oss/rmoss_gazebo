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

#include "rmoss_gz_base/gimbal_controller.hpp"

#include <memory>
#include <string>
#include <algorithm>

namespace rmoss_gz_base
{

GimbalController::GimbalController(
  rclcpp::Node::SharedPtr node,
  Actuator<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_vel_actuator,
  Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_pos_sensor,
  const std::string & controller_name)
: node_(node), gimbal_vel_actuator_(gimbal_vel_actuator), gimbal_pos_sensor_(gimbal_pos_sensor)
{
  // parameter
  declare_pid_parameter(node_, controller_name + ".pitch_pid");
  declare_pid_parameter(node_, controller_name + ".yaw_pid");
  get_pid_parameter(node_, controller_name + ".pitch_pid", picth_pid_param_);
  get_pid_parameter(node_, controller_name + ".yaw_pid", yaw_pid_param_);
  set_pitch_pid(picth_pid_param_);
  set_yaw_pid(yaw_pid_param_);
  // sensor callback
  gimbal_pos_sensor->add_callback(
    [this](const rmoss_interfaces::msg::Gimbal & data, const rclcpp::Time & /*stamp*/) {
      cur_yaw_ = data.yaw;
      cur_pitch_ = data.pitch;
    });
  // ros pub and sub
  using namespace std::placeholders;
  auto ros_gimbal_cmd_topic = "robot_base/gimbal_cmd";
  auto ros_gimbal_state_topic = "robot_base/gimbal_state";
  ros_gimbal_state_pub_ = node_->create_publisher<rmoss_interfaces::msg::Gimbal>(
    ros_gimbal_state_topic, 10);
  ros_gimbal_cmd_sub_ = node_->create_subscription<rmoss_interfaces::msg::GimbalCmd>(
    ros_gimbal_cmd_topic, 10, std::bind(&GimbalController::gimbal_cb, this, _1));
  // timer
  int pid_rate = 100;
  pid_period_ = std::chrono::milliseconds(1000 / pid_rate);
  controller_timer_ = node_->create_wall_timer(
    pid_period_,
    std::bind(&GimbalController::update, this));
  int publish_rate = 10;
  gimbal_state_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(1000 / publish_rate),
    std::bind(&GimbalController::gimbal_state_timer_cb, this));
}

void GimbalController::update()
{
  rmoss_interfaces::msg::Gimbal cmd;
  // pid for pitch
  double pitch_err = cur_pitch_ - target_pitch_;
  cmd.pitch = picth_pid_.Update(pitch_err, pid_period_);
  // pid for yaw
  double yaw_err = cur_yaw_ - target_yaw_;
  cmd.yaw = yaw_pid_.Update(yaw_err, pid_period_);
  // set CMD
  gimbal_vel_actuator_->set(cmd);
}

void GimbalController::gimbal_state_timer_cb()
{
  rmoss_interfaces::msg::Gimbal gimbal_pos;
  gimbal_pos.pitch = cur_pitch_;
  gimbal_pos.yaw = cur_yaw_;
  ros_gimbal_state_pub_->publish(gimbal_pos);
}

void GimbalController::gimbal_cb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg)
{
  // for pitch
  if (msg->pitch_type == msg->ABSOLUTE_ANGLE) {
    target_pitch_ = msg->position.pitch;
  } else if (msg->pitch_type == msg->RELATIVE_ANGLE) {
    target_pitch_ = cur_pitch_ + msg->position.pitch;
  } else {
    RCLCPP_WARN(node_->get_logger(), "pitch cmd type[%d] isn't supported!", msg->pitch_type);
  }
  // limitation for pitch
  target_pitch_ = std::min(target_pitch_, 1.0);
  target_pitch_ = std::max(target_pitch_, -1.0);
  // for yaw
  if (msg->yaw_type == msg->ABSOLUTE_ANGLE) {
    target_yaw_ = msg->position.yaw;
  } else if (msg->yaw_type == msg->RELATIVE_ANGLE) {
    target_yaw_ = cur_yaw_ + msg->position.yaw;
  } else {
    RCLCPP_WARN(node_->get_logger(), "yaw cmd type[%d] isn't supported!", msg->yaw_type);
  }
}

void GimbalController::set_yaw_pid(struct PidParam pid_param)
{
  yaw_pid_.Init(
    pid_param.p, pid_param.i, pid_param.d, pid_param.imax,
    pid_param.imin, pid_param.cmdmax, pid_param.cmdmin, pid_param.offset);
}
void GimbalController::set_pitch_pid(struct PidParam pid_param)
{
  picth_pid_.Init(
    pid_param.p, pid_param.i, pid_param.d, pid_param.imax,
    pid_param.imin, pid_param.cmdmax, pid_param.cmdmin, pid_param.offset);
}

void GimbalController::reset()
{
  target_pitch_ = 0;
  target_yaw_ = 0;
}

}  // namespace rmoss_gz_base
