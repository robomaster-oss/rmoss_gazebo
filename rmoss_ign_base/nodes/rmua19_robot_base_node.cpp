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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rmoss_ign_base/advanced_chassis_controller.hpp"
#include "rmoss_ign_base/advanced_gimbal_controller.hpp"
#include "rmoss_ign_base/shooter_controller.hpp"
#include "rmoss_ign_base/gimbal_state_publisher.hpp"
#include "rmoss_ign_base/pid.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<rclcpp::Node>("robot_base");
  auto ign_node = std::make_shared<ignition::transport::Node>();

  // parameters
  std::string world_name, robot_name;
  ros_node->declare_parameter("world_name", "default");
  world_name = ros_node->get_parameter("world_name").as_string();
  ros_node->declare_parameter("robot_name", "standard_robot");
  robot_name = ros_node->get_parameter("robot_name").as_string();

  // topic string
  std::string ign_chassis_cmd_topic = "/" + robot_name + "/cmd_vel";
  std::string ign_pitch_cmd_topic = "/model/" + robot_name +
    "/joint/gimbal_pitch_joint/cmd_vel";
  std::string ign_yaw_cmd_topic = "/model/" + robot_name + "/joint/gimbal_yaw_joint/cmd_vel";
  std::string ign_joint_state_topic = "/world/" + world_name + "/model/" + robot_name +
    "/joint_state";
  std::string ign_gimbal_imu_topic = "/world/" + world_name + "/model/" + robot_name +
    "/link/gimbal_pitch/sensor/gimbal_imu/imu";
  std::string ign_shooter_cmd_topic = "/" + robot_name + "/small_shooter/shoot";

  // pid parameters
  rmoss_ign_base::PidParam picth_pid_param, yaw_pid_param, chassis_pid_param;
  rmoss_ign_base::declare_pid_parameter(ros_node, "pitch_pid");
  rmoss_ign_base::declare_pid_parameter(ros_node, "yaw_pid");
  rmoss_ign_base::declare_pid_parameter(ros_node, "chassis_pid");
  rmoss_ign_base::get_pid_parameter(ros_node, "pitch_pid", picth_pid_param);
  rmoss_ign_base::get_pid_parameter(ros_node, "yaw_pid", yaw_pid_param);
  rmoss_ign_base::get_pid_parameter(ros_node, "chassis_pid", chassis_pid_param);

  // create ign gimbal sensors (joint encoder and imu)
  auto ign_gimbal_encoder = std::make_shared<rmoss_ign_base::IgnJointEncoder>(
    ign_node, ign_joint_state_topic);
  auto ign_gimbal_imu = std::make_shared<rmoss_ign_base::IgnImu>(ign_node, ign_gimbal_imu_topic);
  // create ign basic controller cmd (chassis cmd and gimbal cmd)
  auto ign_chassis_cmd = std::make_shared<rmoss_ign_base::IgnChassisCmd>(ign_node, ign_chassis_cmd_topic);
  auto ign_gimbal_cmd = std::make_shared<rmoss_ign_base::IgnGimbalCmd>(
    ign_node, ign_pitch_cmd_topic, ign_yaw_cmd_topic);

  // create chassis controller
  auto chassis_controller = std::make_shared<rmoss_ign_base::AdvancedChassisController>(
    ros_node, "chassis_cmd", ign_chassis_cmd, ign_gimbal_encoder);
  chassis_controller->set_chassis_pid(chassis_pid_param);
  chassis_controller->set_control_mode(true);

  // create gimbal controller
  auto gimbal_controller = std::make_shared<rmoss_ign_base::AdvancedGimbalController>(
    ros_node, "gimbal_cmd", ign_gimbal_cmd, ign_gimbal_encoder, ign_gimbal_imu);
  gimbal_controller->set_pitch_pid(picth_pid_param);
  gimbal_controller->set_yaw_pid(yaw_pid_param);

  // create shooter controller
  auto shooter_controller = std::make_shared<rmoss_ign_base::ShooterController>(
    ros_node, ign_node, "shoot_cmd", ign_shooter_cmd_topic);

  // create gimbal state publisher
  auto gimbal_publisher = std::make_shared<rmoss_ign_base::GimbalStatePublisher>(
    ros_node, "gimbal_state", ign_gimbal_imu, 50);

  // run node until it's exited
  rclcpp::spin(ros_node);
  // clean up
  rclcpp::shutdown();
  return 0;
}
