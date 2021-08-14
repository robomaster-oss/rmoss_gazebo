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
#include "rmoss_interfaces/msg/chassis_cmd.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rmoss_ign_base/ign_chassis_cmd.hpp"
#include "rmoss_ign_base/ign_joint_encoder.hpp"
#include "rmoss_ign_base/ign_imu.hpp"
#include "rmoss_ign_base/pid.hpp"

namespace rmoss_ign_base {

class AdvancedChassisController{
public:
    AdvancedChassisController(const rclcpp::Node::SharedPtr& nh, 
        const std::string& ros_chassis_cmd_topic,
        std::shared_ptr<IgnChassisCmd> &ign_chassis_cmd,
        std::shared_ptr<IgnJointEncoder> &ign_gimbal_encoder);
    ~AdvancedChassisController() {};
public:
    void setChassisPid(struct PidParam pid_param);
    void setControlMode(bool follow_mode_flag){ follow_mode_flag_=follow_mode_flag; }
private:
    void chassisCb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg);
    void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void update();
private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<rmoss_interfaces::msg::ChassisCmd>::SharedPtr ros_chassis_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ros_cmd_vel_sub_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_;
    rclcpp::TimerBase::SharedPtr controller_timer_;
    // ignition tool
    std::shared_ptr<IgnChassisCmd> ign_chassis_cmd_;
    std::shared_ptr<IgnJointEncoder> ign_gimbal_encoder_;
    // target data
    double target_vx_;
    double target_vy_;
    double target_w_;
    std::mutex msg_mut_;
    // pid and pid parameter
    ignition::math::PID chassis_pid_;
    PidParam chassis_pid_param_;
    // flag
    bool update_pid_flag_{false};
    bool follow_mode_flag_{true};
};


}

#endif //RMOSS_IGN_BASE_CHASSIS_GIMBAL_CONTROLLER_H