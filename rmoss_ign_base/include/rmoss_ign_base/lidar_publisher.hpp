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

#ifndef RMOSS_IGN_BASE__LIDAR_PUBLISHER_HPP_
#define RMOSS_IGN_BASE__LIDAR_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "ignition/transport/Node.hh"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace rmoss_ign_base
{

class LidarPublisher
{
public:
  LidarPublisher(
    rclcpp::Node::SharedPtr ros_node,
    std::shared_ptr<ignition::transport::Node> ign_node,
    const std::string & ign_lidar_topic,
    const std::string & ros_lidar_topic = "scan",
    int update_rate = 30);
  ~LidarPublisher() {}

  void set_frame_id(const std::string & frame_id);

private:
  void ign_lidar_cb(const ignition::msgs::LaserScan & msg);
  void timer_callback();

private:
  rclcpp::Node::SharedPtr ros_node_;
  std::shared_ptr<ignition::transport::Node> ign_node_;
  // ros pub and sub
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // sensor data
  std::mutex msg_mut_;
  ignition::msgs::LaserScan laser_msg_;
  rclcpp::Time laser_msg_time_;
  std::string frame_id_;
  bool init_frame_{false};
};
}  // namespace rmoss_ign_base
#endif  // RMOSS_IGN_BASE__LIDAR_PUBLISHER_HPP_
