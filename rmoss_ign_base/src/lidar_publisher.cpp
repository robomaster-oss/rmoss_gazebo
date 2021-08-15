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

#include "rmoss_ign_base/lidar_publisher.hpp"

#include <memory>
#include <string>

#include "ros_ign_bridge/convert.hpp"
namespace rmoss_ign_base
{

LidarPublisher::LidarPublisher(
  rclcpp::Node::SharedPtr ros_node,
  std::shared_ptr<ignition::transport::Node> ign_node,
  const std::string & ign_lidar_topic,
  const std::string & ros_lidar_topic,
  int update_rate)
: ros_node_(ros_node), ign_node_(ign_node)
{
  ign_node_->Subscribe(ign_lidar_topic, &LidarPublisher::ign_lidar_cb, this);
  laser_pub_ = ros_node_->create_publisher<sensor_msgs::msg::LaserScan>(ros_lidar_topic, 10);
  auto period = std::chrono::microseconds(1000000 / update_rate);
  timer_ = ros_node_->create_wall_timer(period, std::bind(&LidarPublisher::timer_callback, this));
}

void LidarPublisher::set_frame_id(const std::string & frame_id)
{
  frame_id_ = frame_id;
}

void LidarPublisher::ign_lidar_cb(const ignition::msgs::LaserScan & msg)
{
  std::lock_guard<std::mutex> lock(msg_mut_);
  laser_msg_ = msg;
  // laser_msg_time_ = ros_node_->get_clock()->now();
}

void LidarPublisher::timer_callback()
{
  std::lock_guard<std::mutex> lock(msg_mut_);
  sensor_msgs::msg::LaserScan ros_msg;
  ros_ign_bridge::convert_ign_to_ros(laser_msg_, ros_msg);
  // double current_angle = ros_msg.angle_min;
  // for(unsigned int i = 0; i < ros_msg.ranges.size(); ++i){
  //     if((current_angle > 1.57) && (current_angle < 4.71 )){
  //         ros_msg.ranges[i] = ros_msg.range_max + 1.0;
  //     }
  //     current_angle += ros_msg.angle_increment;
  // }
  if (!frame_id_.empty()) {
    ros_msg.header.frame_id = frame_id_;
  }
  ros_msg.header.stamp = ros_node_->get_clock()->now();
  laser_pub_->publish(ros_msg);
}

}  // namespace rmoss_ign_base
