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

#ifndef RMOSS_GZ_BRIDGE__POSE_BRIDGE_NODE_HPP_
#define RMOSS_GZ_BRIDGE__POSE_BRIDGE_NODE_HPP_

#include <thread>
#include <memory>
#include <string>

#include "ignition/transport/Node.hh"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace rmoss_gz_bridge
{
// Node wrapper for Rmua19RobotBaseNode
class PoseBridgeNode
{
public:
  explicit PoseBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

public:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

  void set_pose_cb(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
  void gz_pose_cb(const ignition::msgs::Pose_V & msg);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  // ros sub
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr set_pose_sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pose_pub_;
  // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_control_sub_;
  std::string gz_service_name_;
  bool robot_filter_;
};

}  // namespace rmoss_gz_bridge

#endif  // RMOSS_GZ_BRIDGE__POSE_BRIDGE_NODE_HPP_
