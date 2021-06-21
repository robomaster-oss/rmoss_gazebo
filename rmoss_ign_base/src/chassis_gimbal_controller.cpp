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
#include "rmoss_ign_base/chassis_gimbal_controller.hpp"

using namespace std;
using namespace rmoss_ign_base;

ChassisGimbalController::ChassisGimbalController(const rclcpp::Node::SharedPtr& nh, 
        const std::string& ros_chassis_cmd_topic,
        const std::string& ros_gimbal_cmd_topic,
        std::shared_ptr<IgnChassisCmdPublisher> &ign_chassis_cmd_publisher,
        std::shared_ptr<IgnGimbalCmdPublisher> &ign_gimbal_cmd_publisher,
        std::shared_ptr<IgnJointEncoder> &ign_gimbal_encoder,
        std::shared_ptr<IgnImu> &ign_gimbal_imu){
    //set handler
    nh_=nh;
    ign_chassis_cmd_publisher_=ign_chassis_cmd_publisher;
    ign_gimbal_cmd_publisher_=ign_gimbal_cmd_publisher;
    ign_gimbal_encoder_=ign_gimbal_encoder;
    ign_gimbal_imu_=ign_gimbal_imu;
    //init pid
    update_pid_flag_ = true;
    //ros sub
    ros_chassis_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::ChassisCmd>(ros_chassis_cmd_topic,
        10, std::bind(&ChassisGimbalController::chassisCb, this, std::placeholders::_1));
    ros_gimbal_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::GimbalCmd>(ros_gimbal_cmd_topic,
        10, std::bind(&ChassisGimbalController::gimbalCb, this, std::placeholders::_1));
    //timer and set_parameters callback
    auto period = std::chrono::microseconds(1000000 / 100);
    callback_group_ = nh_->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    controller_timer_ = nh_->create_wall_timer(period, std::bind(&ChassisGimbalController::update, this),callback_group_);
}

void ChassisGimbalController::update(){
    std::lock_guard<std::mutex> lock(msg_mut_);
    auto dt=std::chrono::microseconds(1000000 / 100);
    // check
    if(update_pid_flag_){
        picth_pid_.Init(pitch_pid_param_.p, pitch_pid_param_.i, pitch_pid_param_.d,pitch_pid_param_.imax,
            pitch_pid_param_.imin,pitch_pid_param_.cmdmax, pitch_pid_param_.cmdmin, pitch_pid_param_.offset);
        yaw_pid_.Init(yaw_pid_param_.p, yaw_pid_param_.i, yaw_pid_param_.d, yaw_pid_param_.imax, 
            yaw_pid_param_.imin, yaw_pid_param_.cmdmax, yaw_pid_param_.cmdmin, yaw_pid_param_.offset);
        chassis_pid_.Init(chassis_pid_param_.p, chassis_pid_param_.i, chassis_pid_param_.d, chassis_pid_param_.imax, 
            chassis_pid_param_.imin, chassis_pid_param_.cmdmax, chassis_pid_param_.cmdmin, chassis_pid_param_.offset);
        RCLCPP_INFO(nh_->get_logger(), "update PID!");
        update_pid_flag_=false;
    }
    // pid for pitch 
    double pitch_err = ign_gimbal_imu_->getPitch() - gimbal_cmd_msg_.position.pitch;
    double pitch_cmd = picth_pid_.Update(pitch_err, dt);
    // pid for yaw
    double yaw_err = ign_gimbal_imu_->getYaw() - gimbal_cmd_msg_.position.yaw;
    double yaw_cmd = yaw_pid_.Update(yaw_err, dt);
    //printf("imu:%lf,%lf\n",pitch_imu_angle_,yaw_imu_angle_);
    //printf("data:%lf,%lf,%lf,%lf\n",pitch_err,pitch_cmd,yaw_err,yaw_cmd);
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
    ign_gimbal_cmd_publisher_->publish(pitch_cmd,yaw_cmd);
    ign_chassis_cmd_publisher_->publish(chassis_cmd_msg_.twist.linear.x,
            chassis_cmd_msg_.twist.linear.y,w_cmd); 
}

void ChassisGimbalController::chassisCb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg){
    std::lock_guard<std::mutex> lock(msg_mut_);
    chassis_cmd_msg_ = *msg;
}

void ChassisGimbalController::gimbalCb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg){
    std::lock_guard<std::mutex> lock(msg_mut_);
    gimbal_cmd_msg_ = *msg;
}

void ChassisGimbalController::setYawPid(struct PidParam pid_param){
    yaw_pid_param_ = pid_param;
    update_pid_flag_ = true;
}
void ChassisGimbalController::setPitchPid(struct PidParam pid_param){
    pitch_pid_param_ = pid_param;
    update_pid_flag_ = true;
}
void ChassisGimbalController::setChassisPid(struct PidParam pid_param){
    chassis_pid_param_ = pid_param;
    update_pid_flag_ = true;
}

