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

#ifndef RMOSS_IGN_BASE__GIMBAL_STATE_PUBLISHER_HPP_
#define RMOSS_IGN_BASE__GIMBAL_STATE_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rmoss_interfaces/msg/gimbal_cmd.hpp"
#include "rmoss_ign_base/ign_imu.hpp"

namespace rmoss_ign_base
{

class GimbalStatePublisher
{
public:
  GimbalStatePublisher(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<IgnImu> ign_gimbal_imu,
    unsigned int update_rate,
    const std::string & gimbal_name = "");
  ~GimbalStatePublisher() {}

private:
  void gimbalStateTimerCb();

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<IgnImu> ign_gimbal_imu_;
  // ros pub and sub
  rclcpp::Publisher<rmoss_interfaces::msg::Gimbal>::SharedPtr ros_gimbal_state_pub_;
  rclcpp::TimerBase::SharedPtr gimbal_state_timer_;
};
}  // namespace rmoss_ign_base
#endif  // RMOSS_IGN_BASE__GIMBAL_STATE_PUBLISHER_HPP_
