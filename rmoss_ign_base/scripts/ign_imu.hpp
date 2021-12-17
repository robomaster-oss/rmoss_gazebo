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
#ifndef RMOSS_IGN_BASE__IGN_IMU_HPP_
#define RMOSS_IGN_BASE__IGN_IMU_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "ignition/transport/Node.hh"
#include "hardware_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"

namespace rmoss_ign_base
{

class IgnImu
{
public:
  IgnImu(
    rclcpp::Node::SharedPtr node,
    const std::shared_ptr<ignition::transport::Node> & ign_node,
    const std::string & ign_gimbal_imu_topic);
  ~IgnImu() {}

public:
  Sensor<double>::SharedPtr get_pitch_sensor() {return pitch_sensor_;}
  Sensor<double>::SharedPtr get_yaw_sensor() {return yaw_sensor_;}
  void reset_yaw(double yaw = 0) { yaw_sensor_->update(yaw, node_->get_clock()->now()); }

private:
  void ign_imu_cb(const ignition::msgs::IMU & msg);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> ign_node_;
  // tmp data
  double last_yaw_{0};
  std::mutex msg_mut_;
  // sensor data
  ignition::msgs::IMU imu_msg_;
  double pitch_angle_{0};
  double yaw_angle_{0};
  double last_yaw_angle_{0};
  double continuous_yaw_angle_{0};
  std::shared_ptr<DataSensor<double>> pitch_sensor_;
  std::shared_ptr<DataSensor<double>> yaw_sensor_;
};

class IgnGimbalImu
{
public:
  IgnGimbalImu(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ignition::transport::Node> ign_node,
    const std::string & ign_gimbal_imu_topic);
  ~IgnGimbalImu() {}

public:
  Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr get_position_sensor() {return position_sensor_;}

private:
  void ign_imu_cb(const ignition::msgs::IMU & msg);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> ign_node_;
  // sensor data
  double last_yaw_angle_{0};
  double continuous_yaw_angle_{0};
  rmoss_interfaces::msg::Gimbal cur_position_;
  std::shared_ptr<DataSensor<rmoss_interfaces::msg::Gimbal>> position_sensor_;
};

}  // namespace rmoss_ign_base

#endif  // RMOSS_IGN_BASE__IGN_IMU_HPP_
