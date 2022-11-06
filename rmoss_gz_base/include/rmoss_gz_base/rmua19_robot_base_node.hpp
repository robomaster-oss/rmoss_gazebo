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

#ifndef RMOSS_GZ_BASE__RMUA19_ROBOT_BASE_NODE_HPP_
#define RMOSS_GZ_BASE__RMUA19_ROBOT_BASE_NODE_HPP_

#include <thread>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "rmoss_gz_base/gz_chassis_actuator.hpp"
#include "rmoss_gz_base/gz_gimbal_actuator.hpp"
#include "rmoss_gz_base/gz_shoot_actuator.hpp"
#include "rmoss_gz_base/gz_gimbal_imu.hpp"
#include "rmoss_gz_base/gz_gimbal_encoder.hpp"
#include "rmoss_gz_base/gz_odometry.hpp"
#include "rmoss_gz_base/gz_light_bar_cmd.hpp"

#include "rmoss_gz_base/chassis_controller.hpp"
#include "rmoss_gz_base/gimbal_controller.hpp"
#include "rmoss_gz_base/shooter_controller.hpp"
#include "rmoss_gz_base/odometry_publisher.hpp"
#include "rmoss_interfaces/msg/robot_status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rmoss_gz_base/pid.hpp"

namespace rmoss_gz_base
{
// Node wrapper for Rmua19RobotBaseNode
class Rmua19RobotBaseNode
{
public:
  explicit Rmua19RobotBaseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

public:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

  void robot_status_cb(const rmoss_interfaces::msg::RobotStatus::SharedPtr msg);
  void enable_power_cb(const std_msgs::msg::Bool::SharedPtr msg);
  void enable_control_cb(const std_msgs::msg::Bool::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  // ros sub
  rclcpp::Subscription<rmoss_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_power_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_control_sub_;
  // ign actuator moudule
  std::shared_ptr<rmoss_gz_base::IgnChassisActuator> chassis_actuator_;
  std::shared_ptr<rmoss_gz_base::IgnGimbalActuator> gimbal_vel_actuator_;
  std::shared_ptr<rmoss_gz_base::IgnShootActuator> shoot_actuator_;
  std::shared_ptr<rmoss_gz_base::IgnLightBarCmd> gz_light_bar_cmd_;
  // ign sensor moudule
  std::shared_ptr<rmoss_gz_base::IgnGimbalEncoder> gz_gimbal_encoder_;
  std::shared_ptr<rmoss_gz_base::IgnGimbalImu> gz_gimbal_imu_;
  std::shared_ptr<rmoss_gz_base::IgnOdometry> gz_chassis_odometry_;
  // ros controller/publisher wrapper
  std::shared_ptr<rmoss_gz_base::ChassisController> chassis_controller_;
  std::shared_ptr<rmoss_gz_base::GimbalController> gimbal_controller_;
  std::shared_ptr<rmoss_gz_base::ShooterController> shooter_controller_;
  std::shared_ptr<rmoss_gz_base::OdometryPublisher> odometry_publisher_;
  //
  bool is_red_;
};

}  // namespace rmoss_gz_base

#endif  // RMOSS_GZ_BASE__RMUA19_ROBOT_BASE_NODE_HPP_
