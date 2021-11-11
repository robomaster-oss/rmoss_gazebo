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

#ifndef RMOSS_IGN_BASE__ODOMETRY_PUBLISHER_HPP_
#define RMOSS_IGN_BASE__ODOMETRY_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "ignition/transport/Node.hh"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace rmoss_ign_base
{

class OdometryPublisher
{
public:
  OdometryPublisher(
    rclcpp::Node::SharedPtr ros_node,
    std::shared_ptr<ignition::transport::Node> ign_node,
    const std::string & ign_odom_topic,
    int update_rate = 50,
    bool publish_tf = true,
    const std::string & odometry_name = "");
  ~OdometryPublisher() {}

  void set_frame_id(const std::string & frame_id);
  void set_child_frame_id(const std::string & child_frame_id);
  void set_footprint(bool use_footprint) {use_footprint_ = use_footprint;}

private:
  void ign_odometry_cb(const ignition::msgs::Odometry & msg);
  void timer_callback();

private:
  rclcpp::Node::SharedPtr ros_node_;
  std::shared_ptr<ignition::transport::Node> ign_node_;
  // ros pub and sub
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  // sensor data
  std::mutex msg_mut_;
  ignition::msgs::Odometry odom_msg_;
  std::string frame_id_;
  std::string child_frame_id_;
  // flag
  bool use_footprint_{false};
  bool publish_tf_{true};
};
}  // namespace rmoss_ign_base
#endif  // RMOSS_IGN_BASE__ODOMETRY_PUBLISHER_HPP_
