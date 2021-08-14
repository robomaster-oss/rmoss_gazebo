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
#include "rmoss_ign_base/advanced_gimbal_controller.hpp"

using namespace std;
using namespace rmoss_ign_base;

AdvancedGimbalController::AdvancedGimbalController(const rclcpp::Node::SharedPtr& nh, 
        const std::string& ros_gimbal_cmd_topic,
        std::shared_ptr<IgnGimbalCmd> &ign_gimbal_cmd,
        std::shared_ptr<IgnJointEncoder> &ign_gimbal_encoder,
        std::shared_ptr<IgnImu> &ign_gimbal_imu){
    //set handler
    nh_=nh;
    ign_gimbal_cmd_=ign_gimbal_cmd;
    ign_gimbal_encoder_=ign_gimbal_encoder;
    ign_gimbal_imu_=ign_gimbal_imu;
    //init pid
    update_pid_flag_ = true;
    //ros sub
    ros_gimbal_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::GimbalCmd>(ros_gimbal_cmd_topic,
        10, std::bind(&AdvancedGimbalController::gimbalCb, this, std::placeholders::_1));
    //timer and set_parameters callback
    auto period = std::chrono::microseconds(1000000 / 100);
    callback_group_ = nh_->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    controller_timer_ = nh_->create_wall_timer(period, std::bind(&AdvancedGimbalController::update, this),callback_group_);
}

void AdvancedGimbalController::update(){
    std::lock_guard<std::mutex> lock(msg_mut_);
    auto dt=std::chrono::microseconds(1000000 / 100);
    // check
    if(update_pid_flag_){
        picth_pid_.Init(pitch_pid_param_.p, pitch_pid_param_.i, pitch_pid_param_.d,pitch_pid_param_.imax,
            pitch_pid_param_.imin,pitch_pid_param_.cmdmax, pitch_pid_param_.cmdmin, pitch_pid_param_.offset);
        yaw_pid_.Init(yaw_pid_param_.p, yaw_pid_param_.i, yaw_pid_param_.d, yaw_pid_param_.imax, 
            yaw_pid_param_.imin, yaw_pid_param_.cmdmax, yaw_pid_param_.cmdmin, yaw_pid_param_.offset);
        RCLCPP_INFO(nh_->get_logger(), "update PID!");
        update_pid_flag_=false;
    }
    // pid for pitch 
    double pitch_err = ign_gimbal_imu_->getPitch() - target_pitch_;
    double pitch_cmd = picth_pid_.Update(pitch_err, dt);
    // pid for yaw
    double yaw_err = ign_gimbal_imu_->getYaw() - target_yaw_;
    double yaw_cmd = yaw_pid_.Update(yaw_err, dt);
    //printf("imu:%lf,%lf\n",ign_gimbal_imu_->getYaw() ,ign_gimbal_imu_->getPitch());
    // publish CMD
    ign_gimbal_cmd_->publish(pitch_cmd,yaw_cmd);
}


void AdvancedGimbalController::gimbalCb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg){
    std::lock_guard<std::mutex> lock(msg_mut_);
    target_yaw_ = msg->position.yaw;
    target_pitch_ = msg->position.pitch;
}

void AdvancedGimbalController::setYawPid(struct PidParam pid_param){
    yaw_pid_param_ = pid_param;
    update_pid_flag_ = true;
}
void AdvancedGimbalController::setPitchPid(struct PidParam pid_param){
    pitch_pid_param_ = pid_param;
    update_pid_flag_ = true;
}

