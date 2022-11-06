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
#ifndef RMOSS_GZ_BASE__GZ_ODOMETRY_HPP_
#define RMOSS_GZ_BASE__GZ_ODOMETRY_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "ignition/transport/Node.hh"
#include "hardware_interface.hpp"
#include "rclcpp/clock.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace rmoss_gz_base
{

class IgnOdometry
{
public:
  IgnOdometry(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ignition::transport::Node> gz_node,
    const std::string & gz_odom_topic);
  ~IgnOdometry() {}

  void enable(bool enable) {enable_ = enable;}
  Sensor<nav_msgs::msg::Odometry>::SharedPtr get_odometry_sensor() {return odometry_sensor_;}

private:
  void gz_odometry_cb(const ignition::msgs::Odometry & msg);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  bool enable_{false};
  std::shared_ptr<DataSensor<nav_msgs::msg::Odometry>> odometry_sensor_;
};

}  // namespace rmoss_gz_base

#endif  // RMOSS_GZ_BASE__GZ_ODOMETRY_HPP_
