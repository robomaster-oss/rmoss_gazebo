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

#ifndef RMOSS_GZ_BASE__GZ_GIMBAL_ACTUATOR_HPP_
#define RMOSS_GZ_BASE__GZ_GIMBAL_ACTUATOR_HPP_

#include <memory>
#include <string>

#include "ignition/transport/Node.hh"
#include "hardware_interface.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"

namespace rmoss_gz_base
{

class IgnGimbalActuator : public Actuator<rmoss_interfaces::msg::Gimbal>
{
public:
  IgnGimbalActuator(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ignition::transport::Node> gz_node,
    const std::string & gz_pitch_topic,
    const std::string & gz_yaw_topic);

  void set(const rmoss_interfaces::msg::Gimbal & data) override;
  void enable(bool enable) {enable_ = enable;}

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  std::unique_ptr<ignition::transport::Node::Publisher> gz_pitch_pub_;
  std::unique_ptr<ignition::transport::Node::Publisher> gz_yaw_pub_;
  bool enable_{false};
};

}  // namespace rmoss_gz_base

#endif  // RMOSS_GZ_BASE__GZ_GIMBAL_ACTUATOR_HPP_
