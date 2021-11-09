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

#include "rmoss_ign_base/gimbal_controller.hpp"

#include <memory>
#include <string>

namespace rmoss_ign_base
{

GimbalController::GimbalController(
  const rclcpp::Node::SharedPtr & nh,
  const std::string & ros_gimbal_cmd_topic,
  std::shared_ptr<IgnGimbalCmd> & ign_gimbal_cmd,
  std::shared_ptr<IgnJointEncoder> & ign_gimbal_encoder,
  std::shared_ptr<IgnImu> & ign_gimbal_imu)
{
  // set handler
  nh_ = nh;
  ign_gimbal_cmd_ = ign_gimbal_cmd;
  ign_gimbal_encoder_ = ign_gimbal_encoder;
  ign_gimbal_imu_ = ign_gimbal_imu;
  // init pid
  update_pid_flag_ = true;
  // ros sub
  ros_gimbal_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::GimbalCmd>(
    ros_gimbal_cmd_topic,
    10, std::bind(&GimbalController::gimbal_cb, this, std::placeholders::_1));
  // timer and set_parameters callback
  auto period = std::chrono::milliseconds(10);
  callback_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  controller_timer_ = nh_->create_wall_timer(
    period, std::bind(&GimbalController::update, this), callback_group_);
}

void GimbalController::update()
{
  std::lock_guard<std::mutex> lock(msg_mut_);
  auto dt = std::chrono::milliseconds(10);
  // check
  if (update_pid_flag_) {
    picth_pid_.Init(
      pitch_pid_param_.p, pitch_pid_param_.i, pitch_pid_param_.d, pitch_pid_param_.imax,
      pitch_pid_param_.imin, pitch_pid_param_.cmdmax, pitch_pid_param_.cmdmin,
      pitch_pid_param_.offset);
    yaw_pid_.Init(
      yaw_pid_param_.p, yaw_pid_param_.i, yaw_pid_param_.d, yaw_pid_param_.imax,
      yaw_pid_param_.imin, yaw_pid_param_.cmdmax, yaw_pid_param_.cmdmin, yaw_pid_param_.offset);
    RCLCPP_INFO(nh_->get_logger(), "update PID!");
    update_pid_flag_ = false;
  }
  // pid for pitch
  double pitch_err = ign_gimbal_imu_->get_pitch() - target_pitch_;
  double pitch_cmd = picth_pid_.Update(pitch_err, dt);
  // pid for yaw
  double yaw_err = ign_gimbal_imu_->get_yaw() - target_yaw_;
  double yaw_cmd = yaw_pid_.Update(yaw_err, dt);
  // printf("imu:%lf,%lf\n",ign_gimbal_imu_->get_yaw() ,ign_gimbal_imu_->get_pitch());
  // printf("imu:%lf,%lf\n",ign_gimbal_imu_->get_yaw() ,target_yaw_);
  // publish CMD
  ign_gimbal_cmd_->publish(pitch_cmd, yaw_cmd);
}


void GimbalController::gimbal_cb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(msg_mut_);
  target_yaw_ = msg->position.yaw;
  target_pitch_ = msg->position.pitch;
  if (target_pitch_ > 1) {
    target_pitch_ = 1;
  }
  if (target_pitch_ < -1) {
    target_pitch_ = -1;
  }
}

void GimbalController::set_yaw_pid(struct PidParam pid_param)
{
  yaw_pid_param_ = pid_param;
  update_pid_flag_ = true;
}
void GimbalController::set_pitch_pid(struct PidParam pid_param)
{
  pitch_pid_param_ = pid_param;
  update_pid_flag_ = true;
}

}  // namespace rmoss_ign_base
