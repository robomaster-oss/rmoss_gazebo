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

#include "rmoss_ign_base/odometry_publisher.hpp"

#include <memory>
#include <string>

#include "ros_ign_bridge/convert.hpp"

namespace rmoss_ign_base
{

OdometryPublisher::OdometryPublisher(
  rclcpp::Node::SharedPtr ros_node,
  std::shared_ptr<ignition::transport::Node> ign_node,
  const std::string & ign_odom_topic,
  const std::string & ros_odom_topic,
  int update_rate,
  bool publish_tf)
: ros_node_(ros_node), ign_node_(ign_node), publish_tf_(publish_tf)
{
  // create ros pub and timer
  odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(ros_odom_topic, 10);
  ign_node_->Subscribe(ign_odom_topic, &OdometryPublisher::ign_odometry_cb, this);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);
  auto period = std::chrono::microseconds(1000000 / update_rate);
  timer_ = ros_node_->create_wall_timer(
    period, std::bind(&OdometryPublisher::timer_callback, this));
}

void OdometryPublisher::ign_odometry_cb(const ignition::msgs::Odometry & msg)
{
  std::lock_guard<std::mutex> lock(msg_mut_);
  if (frame_id_.empty() || child_frame_id_.empty()) {
    for (auto i = 0; i < msg.header().data_size(); ++i) {
      auto aPair = msg.header().data(i);
      if (aPair.key() == "frame_id" && aPair.value_size() > 0 && frame_id_.empty()) {
        frame_id_ = aPair.value(0);
      }
      if (aPair.key() == "child_frame_id" && aPair.value_size() > 0 &&
        child_frame_id_.empty())
      {
        child_frame_id_ = aPair.value(0);
      }
    }
  }
  odom_msg_ = msg;
}

void OdometryPublisher::set_frame_id(const std::string & frame_id)
{
  frame_id_ = frame_id;
}

void OdometryPublisher::set_child_frame_id(const std::string & child_frame_id)
{
  child_frame_id_ = child_frame_id;
}

void OdometryPublisher::timer_callback()
{
  std::lock_guard<std::mutex> lock(msg_mut_);
  auto time = ros_node_->get_clock()->now();
  // odom
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = frame_id_;
  odom_msg.header.stamp = time;
  odom_msg.child_frame_id = child_frame_id_;
  ros_ign_bridge::convert_ign_to_ros(odom_msg_.pose(), odom_msg.pose.pose);
  ros_ign_bridge::convert_ign_to_ros(odom_msg_.twist(), odom_msg.twist.twist);
  if (use_footprint_) {
    odom_msg.pose.pose.position.z = 0;
  }
  odom_pub_->publish(odom_msg);
  // tf
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = frame_id_;
    tf_msg.header.stamp = time;
    tf_msg.child_frame_id = child_frame_id_;
    tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
    tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

}  // namespace rmoss_ign_base
