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
#include "rmoss_ign_base/advanced_chassis_controller.hpp"

#include <memory>
#include <string>


namespace rmoss_ign_base
{
AdvancedChassisController::AdvancedChassisController(
  const rclcpp::Node::SharedPtr & node,
  const std::string & ros_chassis_cmd_topic,
  std::shared_ptr<IgnChassisCmd> & ign_chassis_cmd,
  std::shared_ptr<IgnJointEncoder> & ign_gimbal_encoder)
{
  // set handler
  node_ = node;
  ign_chassis_cmd_ = ign_chassis_cmd;
  ign_gimbal_encoder_ = ign_gimbal_encoder;
  // init pid
  update_pid_flag_ = true;
  // ros sub
  ros_chassis_cmd_sub_ = node_->create_subscription<rmoss_interfaces::msg::ChassisCmd>(
    ros_chassis_cmd_topic,
    10, std::bind(&AdvancedChassisController::chassis_cb, this, std::placeholders::_1));
  ros_cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    10, std::bind(&AdvancedChassisController::cmd_vel_cb, this, std::placeholders::_1));
  // timer and set_parameters callback
  auto period = std::chrono::microseconds(1000000 / 100);
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  controller_timer_ =
    node_->create_wall_timer(
    period, std::bind(
      &AdvancedChassisController::update,
      this), callback_group_);
}

void AdvancedChassisController::update()
{
  std::lock_guard<std::mutex> lock(msg_mut_);
  auto dt = std::chrono::microseconds(1000000 / 100);
  // check
  if (update_pid_flag_) {
    chassis_pid_.Init(
      chassis_pid_param_.p, chassis_pid_param_.i, chassis_pid_param_.d, chassis_pid_param_.imax,
      chassis_pid_param_.imin, chassis_pid_param_.cmdmax, chassis_pid_param_.cmdmin,
      chassis_pid_param_.offset);
    RCLCPP_INFO(node_->get_logger(), "update PID!");
    update_pid_flag_ = false;
  }
  double w_cmd = 0;
  if (follow_mode_flag_) {
    // follow mode
    double w_err = -ign_gimbal_encoder_->get_yaw();
    w_cmd = chassis_pid_.Update(w_err, dt);
  } else {
    // independent mode
    w_cmd = target_w_;
  }
  // publish CMD
  ign_chassis_cmd_->publish(target_vx_, target_vy_, w_cmd);
}

void AdvancedChassisController::chassis_cb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(msg_mut_);
  target_vx_ = msg->twist.linear.x;
  target_vy_ = msg->twist.linear.y;
  target_w_ = msg->twist.angular.z;
}

void AdvancedChassisController::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(msg_mut_);
  target_vx_ = msg->linear.x;
  target_vy_ = msg->linear.y;
  target_w_ = msg->angular.z;
}

void AdvancedChassisController::set_chassis_pid(struct PidParam pid_param)
{
  chassis_pid_param_ = pid_param;
  update_pid_flag_ = true;
}

}  // namespace rmoss_ign_base
