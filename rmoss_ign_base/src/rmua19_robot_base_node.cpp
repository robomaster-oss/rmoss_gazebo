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

#include "rmoss_ign_base/rmua19_robot_base_node.hpp"

#include <thread>
#include <memory>
#include <string>

namespace rmoss_ign_base
{

Rmua19RobotBaseNode::Rmua19RobotBaseNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("robot_base", options);
  ign_node_ = std::make_shared<ignition::transport::Node>();
  // parameters
  std::string world_name, robot_name;
  bool use_odometry = false;
  node_->declare_parameter("world_name", "default");
  node_->declare_parameter("robot_name", "red_standard_robot1");
  node_->declare_parameter("use_odometry", use_odometry);
  node_->get_parameter("robot_name", robot_name);
  node_->get_parameter("world_name", world_name);
  node_->get_parameter("use_odometry", use_odometry);
  is_red_ = (robot_name.find("blue") == std::string::npos);
  // ign topic string
  std::string ign_chassis_cmd_topic = "/" + robot_name + "/cmd_vel";
  std::string ign_pitch_cmd_topic = "/model/" + robot_name + "/joint/gimbal_pitch_joint/cmd_vel";
  std::string ign_yaw_cmd_topic = "/model/" + robot_name + "/joint/gimbal_yaw_joint/cmd_vel";
  std::string ign_joint_state_topic = "/world/" + world_name + "/model/" + robot_name +
    "/joint_state";
  std::string ign_gimbal_imu_topic = "/world/" + world_name + "/model/" + robot_name +
    "/link/gimbal_pitch/sensor/gimbal_imu/imu";
  std::string ign_light_bar_cmd_topic = "/" + robot_name + "/color/set_state";
  // pid parameters
  rmoss_ign_base::PidParam picth_pid_param, yaw_pid_param, chassis_pid_param;
  rmoss_ign_base::declare_pid_parameter(node_, "gimbal_pitch_pid");
  rmoss_ign_base::declare_pid_parameter(node_, "gimbal_yaw_pid");
  rmoss_ign_base::declare_pid_parameter(node_, "chassis_follow_pid");
  rmoss_ign_base::get_pid_parameter(node_, "gimbal_pitch_pid", picth_pid_param);
  rmoss_ign_base::get_pid_parameter(node_, "gimbal_yaw_pid", yaw_pid_param);
  rmoss_ign_base::get_pid_parameter(node_, "chassis_follow_pid", chassis_pid_param);
  // create ign moudule
  ign_gimbal_encoder_ = std::make_shared<rmoss_ign_base::IgnJointEncoder>(
    ign_node_, ign_joint_state_topic);
  ign_gimbal_imu_ = std::make_shared<rmoss_ign_base::IgnImu>(ign_node_, ign_gimbal_imu_topic);
  ign_chassis_cmd_ = std::make_shared<rmoss_ign_base::IgnChassisCmd>(
    ign_node_, ign_chassis_cmd_topic);
  ign_gimbal_cmd_ = std::make_shared<rmoss_ign_base::IgnGimbalCmd>(
    ign_node_, ign_pitch_cmd_topic, ign_yaw_cmd_topic);
  ign_light_bar_cmd_ = std::make_shared<rmoss_ign_base::IgnLightBarCmd>(
    ign_node_, ign_light_bar_cmd_topic);
  // create ros controller and publisher
  chassis_controller_ = std::make_shared<rmoss_ign_base::ChassisController>(
    node_, ign_chassis_cmd_, ign_gimbal_encoder_);
  gimbal_controller_ = std::make_shared<rmoss_ign_base::GimbalController>(
    node_, ign_gimbal_cmd_, ign_gimbal_encoder_, ign_gimbal_imu_, "", 100, 50);
  shooter_controller_ = std::make_shared<rmoss_ign_base::ShooterController>(
    node_, ign_node_, robot_name, "small_shooter");
  if (use_odometry) {
    odometry_publisher_ = std::make_shared<rmoss_ign_base::OdometryPublisher>(
      node_, ign_node_, "/" + robot_name + "/odometry");
    odometry_publisher_->set_child_frame_id(robot_name + "/footprint");
    odometry_publisher_->set_footprint(true);
  }
  // set pid
  chassis_controller_->set_chassis_pid(chassis_pid_param);
  gimbal_controller_->set_pitch_pid(picth_pid_param);
  gimbal_controller_->set_yaw_pid(yaw_pid_param);
  //
  using namespace std::placeholders;

  std::string robot_status_topic = "/referee_system/" + robot_name + "/robot_status";
  robot_status_sub_ = node_->create_subscription<rmoss_interfaces::msg::RobotStatus>(
    robot_status_topic, 10, std::bind(&Rmua19RobotBaseNode::robot_status_cb, this, _1));
  std::string enable_power_topic = "/referee_system/" + robot_name + "/enable_power";
  enable_power_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    enable_power_topic, 10, std::bind(&Rmua19RobotBaseNode::enable_power_cb, this, _1));
  std::string enable_control_topic = "/referee_system/" + robot_name + "/enable_control";
  enable_control_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    enable_control_topic, 10, std::bind(&Rmua19RobotBaseNode::enable_control_cb, this, _1));
}


void Rmua19RobotBaseNode::robot_status_cb(
  const rmoss_interfaces::msg::RobotStatus::SharedPtr msg)
{
  int remain_num = msg->total_projectiles - msg->used_projectiles;
  if (remain_num < 0) {
    remain_num = 0;
  }
  shooter_controller_->updata_remain_num(remain_num);
}

void Rmua19RobotBaseNode::enable_power_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    // enable power
    chassis_controller_->reset();
    gimbal_controller_->reset();
    chassis_controller_->enable_power(true);
    gimbal_controller_->enable_power(true);
    shooter_controller_->enable(true);
    if (is_red_) {
      ign_light_bar_cmd_->set_state(1);
    } else {
      ign_light_bar_cmd_->set_state(2);
    }
  } else {
    // disable power
    chassis_controller_->enable_power(false);
    gimbal_controller_->enable_power(false);
    shooter_controller_->enable(false);
    ign_light_bar_cmd_->set_state(0);
  }
}

void Rmua19RobotBaseNode::enable_control_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    // enable control
    chassis_controller_->enable_control(true);
    gimbal_controller_->enable_control(true);
    shooter_controller_->enable(true);
  } else {
    // disable control
    chassis_controller_->enable_control(false);
    gimbal_controller_->enable_control(false);
    shooter_controller_->enable(false);
  }
}

}  // namespace rmoss_ign_base

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmoss_ign_base::Rmua19RobotBaseNode)
