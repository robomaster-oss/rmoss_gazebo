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
#include "rmoss_gz_base/gz_odometry.hpp"

#include <cmath>
#include <memory>
#include <string>

namespace rmoss_gz_base
{


IgnOdometry::IgnOdometry(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<ignition::transport::Node> gz_node,
  const std::string & gz_odom_topic)
: node_(node), gz_node_(gz_node)
{
  gz_node_->Subscribe(gz_odom_topic, &IgnOdometry::gz_odometry_cb, this);
  odometry_sensor_ = std::make_shared<DataSensor<nav_msgs::msg::Odometry>>();
}

void IgnOdometry::gz_odometry_cb(const ignition::msgs::Odometry & msg)
{
  if (!enable_) {
    return;
  }
  nav_msgs::msg::Odometry odom_msg;
  auto & pose = msg.pose();
  odom_msg.pose.pose.position.x = pose.position().x();
  odom_msg.pose.pose.position.y = pose.position().y();
  odom_msg.pose.pose.position.z = pose.position().z();
  odom_msg.pose.pose.orientation.x = pose.orientation().x();
  odom_msg.pose.pose.orientation.y = pose.orientation().y();
  odom_msg.pose.pose.orientation.z = pose.orientation().z();
  odom_msg.pose.pose.orientation.w = pose.orientation().w();
  odom_msg.twist.twist.linear.x = msg.twist().linear().x();
  odom_msg.twist.twist.linear.y = msg.twist().linear().y();
  odom_msg.twist.twist.angular.z = msg.twist().angular().z();
  odometry_sensor_->update(odom_msg, node_->get_clock()->now());
}

}  // namespace rmoss_gz_base
