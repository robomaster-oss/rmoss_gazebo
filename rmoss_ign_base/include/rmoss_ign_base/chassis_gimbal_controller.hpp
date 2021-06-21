/*******************************************************************************
 *  Copyright (c) 2021 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#ifndef RMOSS_IGN_BASE_CHASSIS_GIMBAL_CONTROLLER_H
#define RMOSS_IGN_BASE_CHASSIS_GIMBAL_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <ignition/math/PID.hh>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "rmoss_interfaces/msg/chassis_cmd.hpp"
#include "rmoss_interfaces/msg/gimbal_cmd.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"

#include "rmoss_ign_base/ign_chassis_cmd_publisher.hpp"
#include "rmoss_ign_base/ign_gimbal_cmd_publisher.hpp"
#include "rmoss_ign_base/ign_joint_encoder.hpp"
#include "rmoss_ign_base/ign_imu.hpp"

namespace rmoss_ign_base {

struct PidParam{
    double p=1;
    double i=0;
    double d=0;
    double imax=1;
    double imin=-1;
    double cmdmax=100;
    double cmdmin=-100;
    double offset=0;
};

class ChassisGimbalController{
public:
    ChassisGimbalController(const rclcpp::Node::SharedPtr& nh, 
        const std::string& ros_chassis_cmd_topic,
        const std::string& ros_gimbal_cmd_topic,
        std::shared_ptr<IgnChassisCmdPublisher> &ign_chassis_cmd_publisher,
        std::shared_ptr<IgnGimbalCmdPublisher> &ign_gimbal_cmd_publisher,
        std::shared_ptr<IgnJointEncoder> &ign_gimbal_encoder,
        std::shared_ptr<IgnImu> &ign_gimbal_imu);
    ~ChassisGimbalController() {};
public:
    void setYawPid(struct PidParam pid_param);
    void setPitchPid(struct PidParam pid_param);
    void setChassisPid(struct PidParam pid_param);
    void setControlMode(bool follow_mode_flag){ follow_mode_flag_=follow_mode_flag; }
private:
    void chassisCb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg);
    void gimbalCb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg);
    void update();
private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<rmoss_interfaces::msg::ChassisCmd>::SharedPtr ros_chassis_cmd_sub_;
    rclcpp::Subscription<rmoss_interfaces::msg::GimbalCmd>::SharedPtr ros_gimbal_cmd_sub_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_;
    rclcpp::TimerBase::SharedPtr controller_timer_;
    // ignition tool
    std::shared_ptr<IgnChassisCmdPublisher> ign_chassis_cmd_publisher_;
    std::shared_ptr<IgnGimbalCmdPublisher> ign_gimbal_cmd_publisher_;
    std::shared_ptr<IgnJointEncoder> ign_gimbal_encoder_;
    std::shared_ptr<IgnImu> ign_gimbal_imu_;
    // target data
    rmoss_interfaces::msg::ChassisCmd chassis_cmd_msg_;
    rmoss_interfaces::msg::GimbalCmd gimbal_cmd_msg_;
    std::mutex msg_mut_;
    // pid and pid parameter
    ignition::math::PID picth_pid_;
    ignition::math::PID yaw_pid_;
    ignition::math::PID chassis_pid_;
    PidParam pitch_pid_param_;
    PidParam yaw_pid_param_;
    PidParam chassis_pid_param_;
    // flag
    bool update_pid_flag_{false};
    bool follow_mode_flag_{true};
};


}

#endif //RMOSS_IGN_BASE_CHASSIS_GIMBAL_CONTROLLER_H