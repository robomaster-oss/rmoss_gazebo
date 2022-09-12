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

#ifndef RMOSS_GZ_BASE__ODOMETRY_PUBLISHER_HPP_
#define RMOSS_GZ_BASE__ODOMETRY_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "hardware_interface.hpp"

namespace rmoss_gz_base
{

class OdometryPublisher
{
public:
  OdometryPublisher(
    rclcpp::Node::SharedPtr node,
    Sensor<nav_msgs::msg::Odometry>::SharedPtr odometry_sensor,
    const std::string & publisher_name = "odometry_publisher");
  ~OdometryPublisher() {}

private:
  void timer_callback();

private:
  rclcpp::Node::SharedPtr node_;
  // ros pub
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  // sensor data
  std::mutex msg_mut_;
  Sensor<nav_msgs::msg::Odometry>::SharedPtr odometry_sensor_;
  nav_msgs::msg::Odometry sensor_msg_;
  std::string frame_id_{"odom"};
  std::string child_frame_id_{"base_link"};
  bool use_footprint_{false};
  bool publish_tf_{true};
};
}  // namespace rmoss_gz_base
#endif  // RMOSS_GZ_BASE__ODOMETRY_PUBLISHER_HPP_
