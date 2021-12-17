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

#ifndef RMOSS_IGN_BASE__IGN_JOINT_ENCODER_HPP_
#define RMOSS_IGN_BASE__IGN_JOINT_ENCODER_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <map>
#include <vector>

#include "ignition/transport/Node.hh"
#include "hardware_interface.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"

namespace rmoss_ign_base
{

class IgnJointEncoder 
{
public:
  IgnJointEncoder(
    rclcpp::Node::SharedPtr node,
    const std::shared_ptr<ignition::transport::Node> & ign_node,
    const std::string & ign_joint_state_topic);
  ~IgnJointEncoder() {}

public:
  bool is_init() {return is_init_;}

  Sensor<double>::SharedPtr get_position_sensor(const std::string & joint_name) {
    if (joint_position_sensors_.find(joint_name) == joint_position_sensors_.end()){
      return nullptr;
    }
    return joint_position_sensors_[joint_name];
  }
  Sensor<double>::SharedPtr get_velocity_sensor(const std::string & joint_name){
    if (joint_velocity_sensors_.find(joint_name) == joint_velocity_sensors_.end()){
      return nullptr;
    }
    return joint_velocity_sensors_[joint_name];
  }
  Sensor<double>::SharedPtr get_effort_sensor(const std::string & joint_name) {
    if (joint_effort_sensors_.find(joint_name) == joint_effort_sensors_.end()){
      return nullptr;
    }
    return joint_effort_sensors_[joint_name];
  }
private:
  void ign_Joint_state_cb(const ignition::msgs::Model & msg);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> ign_node_;
  // sensor data
  ignition::msgs::Model joint_state_msg_;
  std::mutex msg_mut_;
  // info
  std::set<std::string> joint_names_;
  bool is_init_{false};
  std::map<std::string, std::shared_ptr<DataSensor<double>>> joint_position_sensors_;
  std::map<std::string, std::shared_ptr<DataSensor<double>>> joint_velocity_sensors_;
  std::map<std::string, std::shared_ptr<DataSensor<double>>> joint_effort_sensors_;
};


class IgnGimbalEncoder
{
public:
  IgnGimbalEncoder(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ignition::transport::Node> ign_node,
    const std::string & ign_joint_state_topic);
  ~IgnGimbalEncoder() {}

public:
  Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr get_position_sensor() {return position_sensor_;}
  Sensor<rmoss_interfaces::msg::Gimbal>::SharedPtr get_velocity_sensor() {return velocity_sensor_;}

private:
  void ign_Joint_state_cb(const ignition::msgs::Model & msg);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> ign_node_;
  // info
  std::shared_ptr<DataSensor<rmoss_interfaces::msg::Gimbal>> position_sensor_;
  std::shared_ptr<DataSensor<rmoss_interfaces::msg::Gimbal>> velocity_sensor_;
};


}  // namespace rmoss_ign_base

#endif  // RMOSS_IGN_BASE__IGN_JOINT_ENCODER_HPP_
