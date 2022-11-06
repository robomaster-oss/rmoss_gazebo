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
#include "rmoss_gz_base/gz_chassis_actuator.hpp"

#include <memory>
#include <string>

namespace rmoss_gz_base
{


IgnChassisActuator::IgnChassisActuator(
  rclcpp::Node::SharedPtr node,
  const std::shared_ptr<ignition::transport::Node> & gz_node,
  const std::string & gz_chassis_cmd_topic)
: node_(node), gz_node_(gz_node)
{
  gz_chassis_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
    gz_node_->Advertise<ignition::msgs::Twist>(gz_chassis_cmd_topic));
}

void IgnChassisActuator::set(const geometry_msgs::msg::Twist & data)
{
  if (!enable_) {
    return;
  }
  ignition::msgs::Twist gz_msg;
  gz_msg.mutable_linear()->set_x(data.linear.x);
  gz_msg.mutable_linear()->set_y(data.linear.y);
  gz_msg.mutable_angular()->set_z(data.angular.z);
  gz_chassis_cmd_pub_->Publish(gz_msg);
}

}  // namespace rmoss_gz_base
