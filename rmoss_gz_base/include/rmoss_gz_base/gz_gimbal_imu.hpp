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
#ifndef RMOSS_GZ_BASE__GZ_GIMBAL_IMU_HPP_
#define RMOSS_GZ_BASE__GZ_GIMBAL_IMU_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "ignition/transport/Node.hh"
#include "hardware_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"

namespace rmoss_gz_base
{

class IgnGimbalImu
{
public:
  IgnGimbalImu(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ignition::transport::Node> gz_node,
    const std::string & gz_gimbal_imu_topic);
  ~IgnGimbalImu() {}

  void enable(bool enable) {enable_ = enable;}
  Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr get_position_sensor() {return position_sensor_;}

private:
  void gz_imu_cb(const ignition::msgs::IMU & msg);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  bool enable_{false};
  // sensor data
  double last_yaw_angle_{0};
  double continuous_yaw_angle_{0};
  rmoss_interfaces::msg::Gimbal cur_position_;
  std::shared_ptr<DataSensor<rmoss_interfaces::msg::Gimbal>> position_sensor_;
};

}  // namespace rmoss_gz_base

#endif  // RMOSS_GZ_BASE__GZ_GIMBAL_IMU_HPP_
