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
#include "rmoss_ign_base/gimbal_state_publisher.hpp"

#include <memory>
#include <string>

namespace rmoss_ign_base
{
GimbalStatePublisher::GimbalStatePublisher(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<IgnImu> ign_gimbal_imu,
  unsigned int update_rate,
  const std::string & gimbal_name)
{
  // ROS and Ignition node
  node_ = node;
  ign_gimbal_imu_ = ign_gimbal_imu;
  // create ros pub and timer
  std::string ros_gimbal_state_topic = "robot_base/gimbal_state";
  if (gimbal_name != "") {
    ros_gimbal_state_topic = "robot_base/" + gimbal_name + "/gimbal_state";
  }
  ros_gimbal_state_pub_ = node_->create_publisher<rmoss_interfaces::msg::Gimbal>(
    ros_gimbal_state_topic, 10);
  auto period = std::chrono::microseconds(1000000 / update_rate);
  gimbal_state_timer_ = node_->create_wall_timer(
    period, std::bind(&GimbalStatePublisher::gimbalStateTimerCb, this));
}

void GimbalStatePublisher::gimbalStateTimerCb()
{
  rmoss_interfaces::msg::Gimbal gimbal_state;
  gimbal_state.pitch = ign_gimbal_imu_->get_pitch();
  gimbal_state.yaw = ign_gimbal_imu_->get_yaw();
  ros_gimbal_state_pub_->publish(gimbal_state);
}


}  // namespace rmoss_ign_base
