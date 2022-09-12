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

#include "rmoss_gz_base/odometry_publisher.hpp"

#include <memory>
#include <string>

namespace rmoss_gz_base
{

OdometryPublisher::OdometryPublisher(
  rclcpp::Node::SharedPtr node,
  Sensor<nav_msgs::msg::Odometry>::SharedPtr odometry_sensor,
  const std::string & publisher_name)
: node_(node), odometry_sensor_(odometry_sensor)
{
  // parameters
  int rate = 30;
  std::string param_ns = publisher_name + ".";
  node_->declare_parameter(param_ns + "rate", rate);
  node_->declare_parameter(param_ns + "frame_id", frame_id_);
  node_->declare_parameter(param_ns + "child_frame_id", child_frame_id_);
  node_->declare_parameter(param_ns + "publish_tf", publish_tf_);
  node_->declare_parameter(param_ns + "use_footprint", use_footprint_);
  node_->get_parameter(param_ns + "rate", rate);
  node_->get_parameter(param_ns + "frame_id", frame_id_);
  node_->get_parameter(param_ns + "child_frame_id", child_frame_id_);
  node_->get_parameter(param_ns + "publish_tf", publish_tf_);
  node_->get_parameter(param_ns + "use_footprint", use_footprint_);
  // create ros pub and timer
  std::string odom_topic = "robot_base/odom";
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  auto period = std::chrono::microseconds(1000000 / rate);
  timer_ = node_->create_wall_timer(
    period, std::bind(&OdometryPublisher::timer_callback, this));
}

void OdometryPublisher::timer_callback()
{
  // odom
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = frame_id_;
  odom_msg.child_frame_id = child_frame_id_;
  {
    std::lock_guard<std::mutex> lock(msg_mut_);
    odom_msg.header.stamp = sensor_msg_.header.stamp;
    odom_msg.pose = sensor_msg_.pose;
    odom_msg.twist = sensor_msg_.twist;
  }
  if (use_footprint_) {
    odom_msg.pose.pose.position.z = 0;
  }
  odom_pub_->publish(odom_msg);
  // tf
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = odom_msg.header.frame_id;
    tf_msg.header.stamp = odom_msg.header.stamp;
    tf_msg.child_frame_id = odom_msg.child_frame_id;
    tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
    tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

}  // namespace rmoss_gz_base
