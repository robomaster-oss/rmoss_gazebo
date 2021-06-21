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
#include "rmoss_ign_base/chassis_gimbal_controller_base.hpp"

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

class ChassisGimbalController:public ChassisGimbalControllerBase {
public:
    ChassisGimbalController(const rclcpp::Node::SharedPtr& nh);
    ~ChassisGimbalController() {};
private:
    void update();
    rcl_interfaces::msg::SetParametersResult parametersCb(
        const std::vector<rclcpp::Parameter> &parameters);
private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_handle_;
    rclcpp::TimerBase::SharedPtr controller_timer_;
    // pid and pid parameter
    ignition::math::PID picth_pid_;
    ignition::math::PID yaw_pid_;
    ignition::math::PID chassis_pid_;
    PidParam picth_pid_param_;
    PidParam yaw_pid_param_;
    PidParam chassis_pid_param_;
    // flag
    bool update_pid_flag_{false};
    bool follow_mode_flag_{true};
};


}

#endif //RMOSS_IGN_BASE_CHASSIS_GIMBAL_CONTROLLER_H