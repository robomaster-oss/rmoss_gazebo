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
#ifndef RMOSS_IGN_BASE_ADVANCED_GIMBAL_CONTROLLER_H
#define RMOSS_IGN_BASE_ADVANCED_GIMBAL_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include "rmoss_interfaces/msg/gimbal_cmd.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"


#include "rmoss_ign_base/ign_gimbal_cmd.hpp"
#include "rmoss_ign_base/ign_joint_encoder.hpp"
#include "rmoss_ign_base/ign_imu.hpp"
#include "rmoss_ign_base/pid.hpp"

namespace rmoss_ign_base {


class AdvancedGimbalController{
public:
    AdvancedGimbalController(const rclcpp::Node::SharedPtr& nh, 
        const std::string& ros_gimbal_cmd_topic,
        std::shared_ptr<IgnGimbalCmd> &ign_gimbal_cmd,
        std::shared_ptr<IgnJointEncoder> &ign_gimbal_encoder,
        std::shared_ptr<IgnImu> &ign_gimbal_imu);
    ~AdvancedGimbalController() {};
public:
    void setYawPid(struct PidParam pid_param);
    void setPitchPid(struct PidParam pid_param);
private:
    void gimbalCb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg);
    void update();
private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<rmoss_interfaces::msg::GimbalCmd>::SharedPtr ros_gimbal_cmd_sub_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_;
    rclcpp::TimerBase::SharedPtr controller_timer_;
    // ignition tool
    std::shared_ptr<IgnGimbalCmd> ign_gimbal_cmd_;
    std::shared_ptr<IgnJointEncoder> ign_gimbal_encoder_;
    std::shared_ptr<IgnImu> ign_gimbal_imu_;
    // target data
    rmoss_interfaces::msg::GimbalCmd gimbal_cmd_msg_;
    std::mutex msg_mut_;
    // pid and pid parameter
    ignition::math::PID picth_pid_;
    ignition::math::PID yaw_pid_;
    PidParam pitch_pid_param_;
    PidParam yaw_pid_param_;
    // flag
    bool update_pid_flag_{false};
};


}

#endif //RMOSS_IGN_BASE_ADVANCED_GIMBAL_CONTROLLER_H