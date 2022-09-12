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

#include "rmoss_ign_bridge/pose_bridge_node.hpp"

#include <thread>
#include <memory>
#include <string>

#include "ros_ign_bridge/convert.hpp"

namespace rmoss_ign_bridge
{

PoseBridgeNode::PoseBridgeNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("pose_bridge", options);
  ign_node_ = std::make_shared<ignition::transport::Node>();
  // parameters
  std::string world_name;
  node_->declare_parameter("world_name", "default");
  node_->declare_parameter("robot_filter", false);
  node_->get_parameter("world_name", world_name);
  node_->get_parameter("robot_filter", robot_filter_);
  std::string ign_topic = "/world/" + world_name + "/dynamic_pose/info";
  ign_service_name_ = "/world/" + world_name + "/set_pose";
  // get pose from ignition gazebo
  ign_node_->Subscribe(ign_topic, &PoseBridgeNode::ign_pose_cb, this);
  pose_pub_ = node_->create_publisher<tf2_msgs::msg::TFMessage>(
    "/referee_system/ign/pose_info", 10);
  // set pose to ignition gazebo
  using namespace std::placeholders;
  set_pose_sub_ = node_->create_subscription<geometry_msgs::msg::TransformStamped>(
    "/referee_system/ign/set_pose", 10, std::bind(&PoseBridgeNode::set_pose_cb, this, _1));
}

void PoseBridgeNode::ign_pose_cb(const ignition::msgs::Pose_V & msg)
{
  tf2_msgs::msg::TFMessage ros_msg;
  for (auto const & p : msg.pose()) {
    if (robot_filter_) {
      if (p.name().find("robot") == std::string::npos) {
        continue;
      }
    }
    geometry_msgs::msg::TransformStamped tf;
    ros_ign_bridge::convert_ign_to_ros(p, tf.transform);
    tf.child_frame_id = p.name();
    ros_msg.transforms.push_back(tf);
  }
  pose_pub_->publish(ros_msg);
}

void PoseBridgeNode::set_pose_cb(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  // Request message
  ignition::msgs::Pose req;
  ros_ign_bridge::convert_ros_to_ign(msg->transform, req);
  req.set_name(msg->child_frame_id);
  ignition::msgs::Boolean rep;
  bool result;
  bool executed = ign_node_->Request(ign_service_name_, req, 500, rep, result);
  if (executed) {
    if (!result) {
      RCLCPP_ERROR(
        node_->get_logger(), "Ignition Service %s call failed\n %s",
        ign_service_name_.c_str(), req.DebugString().c_str());
    }
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "Ignition Service %s call timed out\n %s",
      ign_service_name_.c_str(), req.DebugString().c_str());
  }
}


}  // namespace rmoss_ign_bridge
