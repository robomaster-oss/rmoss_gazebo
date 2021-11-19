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
#include "rmoss_ign_base/chassis_controller.hpp"

#include <memory>
#include <string>


namespace rmoss_ign_base
{
ChassisController::ChassisController(
  const rclcpp::Node::SharedPtr & node,
  std::shared_ptr<IgnChassisCmd> & ign_chassis_cmd,
  std::shared_ptr<IgnJointEncoder> & ign_gimbal_encoder)
{
  // set handler
  node_ = node;
  ign_chassis_cmd_ = ign_chassis_cmd;
  ign_gimbal_encoder_ = ign_gimbal_encoder;
  // ros sub
  using namespace std::placeholders;
  ros_chassis_cmd_sub_ = node_->create_subscription<rmoss_interfaces::msg::ChassisCmd>(
    "robot_base/chassis_cmd", 10, std::bind(&ChassisController::chassis_cb, this, _1));
  ros_cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&ChassisController::cmd_vel_cb, this, _1));
  // timer and set_parameters callback
  auto period = std::chrono::milliseconds(10);
  controller_timer_ = node_->create_wall_timer(
    period, std::bind(&ChassisController::update, this));
}

void ChassisController::update()
{
  auto dt = std::chrono::milliseconds(10);
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

void ChassisController::chassis_cb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg)
{
  if (!enable_) {
    return;
  }
  if (msg->type == msg->VELOCITY) {
    target_vx_ = msg->twist.linear.x;
    target_vy_ = msg->twist.linear.y;
    target_w_ = msg->twist.angular.z;
    follow_mode_flag_ = false;
  } else if (msg->type == msg->FOLLOW_GIMBAL) {
    target_vx_ = msg->twist.linear.x;
    target_vy_ = msg->twist.linear.y;
    if (!follow_mode_flag_) {
      follow_mode_flag_ = true;
      chassis_pid_.Reset();
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "chassis type[%d] isn't supported!", msg->type);
  }
}

void ChassisController::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!enable_) {
    return;
  }
  target_vx_ = msg->linear.x;
  target_vy_ = msg->linear.y;
  target_w_ = msg->angular.z;
  follow_mode_flag_ = false;
}

void ChassisController::set_chassis_pid(struct PidParam pid_param)
{
  chassis_pid_.Init(
    pid_param.p, pid_param.i, pid_param.d, pid_param.imax,
    pid_param.imin, pid_param.cmdmax, pid_param.cmdmin, pid_param.offset);
}

}  // namespace rmoss_ign_base
