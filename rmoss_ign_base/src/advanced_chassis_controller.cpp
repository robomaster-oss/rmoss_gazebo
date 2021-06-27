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
#include "rmoss_ign_base/advanced_chassis_controller.hpp"

using namespace std;
using namespace rmoss_ign_base;

AdvancedChassisController::AdvancedChassisController(const rclcpp::Node::SharedPtr& nh, 
        const std::string& ros_chassis_cmd_topic,
        std::shared_ptr<IgnChassisCmd> &ign_chassis_cmd,
        std::shared_ptr<IgnJointEncoder> &ign_gimbal_encoder){
    //set handler
    nh_=nh;
    ign_chassis_cmd_=ign_chassis_cmd;
    ign_gimbal_encoder_=ign_gimbal_encoder;
    //init pid
    update_pid_flag_ = true;
    //ros sub
    ros_chassis_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::ChassisCmd>(ros_chassis_cmd_topic,
        10, std::bind(&AdvancedChassisController::chassisCb, this, std::placeholders::_1));
    //timer and set_parameters callback
    auto period = std::chrono::microseconds(1000000 / 100);
    callback_group_ = nh_->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    controller_timer_ = nh_->create_wall_timer(period, std::bind(&AdvancedChassisController::update, this),callback_group_);
}

void AdvancedChassisController::update(){
    std::lock_guard<std::mutex> lock(msg_mut_);
    auto dt=std::chrono::microseconds(1000000 / 100);
    // check
    if(update_pid_flag_){
        chassis_pid_.Init(chassis_pid_param_.p, chassis_pid_param_.i, chassis_pid_param_.d, chassis_pid_param_.imax, 
            chassis_pid_param_.imin, chassis_pid_param_.cmdmax, chassis_pid_param_.cmdmin, chassis_pid_param_.offset);
        RCLCPP_INFO(nh_->get_logger(), "update PID!");
        update_pid_flag_=false;
    }
    double w_cmd=0;
    if(follow_mode_flag_){
        // follow mode
        double w_err = -ign_gimbal_encoder_->getYaw();
        w_cmd = chassis_pid_.Update(w_err, dt);
    }else{
         // independent mode
        w_cmd = chassis_cmd_msg_.twist.angular.z;
    }
    // publish CMD
    ign_chassis_cmd_->publish(chassis_cmd_msg_.twist.linear.x,
            chassis_cmd_msg_.twist.linear.y,w_cmd); 
}

void AdvancedChassisController::chassisCb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg){
    std::lock_guard<std::mutex> lock(msg_mut_);
    chassis_cmd_msg_ = *msg;
}

void AdvancedChassisController::setChassisPid(struct PidParam pid_param){
    chassis_pid_param_ = pid_param;
    update_pid_flag_ = true;
}

