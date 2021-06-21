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
#include "rmoss_ign_base/chassis_gimbal_controller_base.hpp"

using namespace std;
using namespace rmoss_ign_base;

ChassisGimbalController::ChassisGimbalController(const rclcpp::Node::SharedPtr& nh)
            :ChassisGimbalControllerBase(nh){
    nh_=nh;
    //default parameters
    picth_pid_param_.p=10;
    yaw_pid_param_.p=10;
    yaw_pid_param_.i=0.1;
    yaw_pid_param_.d=1;
    chassis_pid_param_.p=1;
    chassis_pid_param_.i=0.1;
    nh_->declare_parameter("pitch_pid_p", picth_pid_param_.p);
    nh_->declare_parameter("pitch_pid_i", picth_pid_param_.i);
    nh_->declare_parameter("pitch_pid_d", picth_pid_param_.d);
    nh_->declare_parameter("yaw_pid_p", yaw_pid_param_.p);
    nh_->declare_parameter("yaw_pid_i", yaw_pid_param_.i);
    nh_->declare_parameter("yaw_pid_d", yaw_pid_param_.d);
    nh_->declare_parameter("chassis_pid_p", chassis_pid_param_.p);
    nh_->declare_parameter("chassis_pid_i", chassis_pid_param_.i);
    nh_->declare_parameter("chassis_pid_d", chassis_pid_param_.d);
    picth_pid_param_.p=nh_->get_parameter("pitch_pid_p").as_double();
    picth_pid_param_.i=nh_->get_parameter("pitch_pid_i").as_double();
    picth_pid_param_.d=nh_->get_parameter("pitch_pid_d").as_double();
    yaw_pid_param_.p=nh_->get_parameter("yaw_pid_p").as_double();
    yaw_pid_param_.i=nh_->get_parameter("yaw_pid_i").as_double();
    yaw_pid_param_.d =nh_->get_parameter("yaw_pid_d").as_double();
    chassis_pid_param_.p=nh_->get_parameter("chassis_pid_p").as_double();
    chassis_pid_param_.i=nh_->get_parameter("chassis_pid_i").as_double();
    chassis_pid_param_.d =nh_->get_parameter("chassis_pid_d").as_double();
    //init pid
    update_pid_flag_ = true;
    //timer and set_parameters callback
    auto period = std::chrono::microseconds(1000000 / 50);
    controller_timer_ = nh_->create_wall_timer(period, std::bind(&ChassisGimbalController::update, this));
    parameter_handle_ = nh_->add_on_set_parameters_callback(
        std::bind(&ChassisGimbalController::parametersCb, this, std::placeholders::_1));
}

void ChassisGimbalController::update(){
    std::lock_guard<std::mutex> lock(msg_mut_);
    auto dt=std::chrono::microseconds(1000000 / 50);
    // check
    if(update_pid_flag_){
        picth_pid_.Init(picth_pid_param_.p, picth_pid_param_.i, picth_pid_param_.d,picth_pid_param_.imax,
            picth_pid_param_.imin,picth_pid_param_.cmdmax, picth_pid_param_.cmdmin, picth_pid_param_.offset);
        yaw_pid_.Init(yaw_pid_param_.p, yaw_pid_param_.i, yaw_pid_param_.d, yaw_pid_param_.imax, 
            yaw_pid_param_.imin, yaw_pid_param_.cmdmax, yaw_pid_param_.cmdmin, yaw_pid_param_.offset);
        chassis_pid_.Init(chassis_pid_param_.p, chassis_pid_param_.i, chassis_pid_param_.d, chassis_pid_param_.imax, 
            chassis_pid_param_.imin, chassis_pid_param_.cmdmax, chassis_pid_param_.cmdmin, chassis_pid_param_.offset);
        RCLCPP_INFO(nh_->get_logger(), "update PID!");
        update_pid_flag_=false;
    }
    // pid for pitch 
    double pitch_err = pitch_imu_angle_ - gimbal_cmd_msg_.position.pitch;
    double pitch_cmd = picth_pid_.Update(pitch_err, dt);
    // pid for yaw
    double yaw_err = yaw_imu_angle_ - gimbal_cmd_msg_.position.yaw;
    double yaw_cmd = yaw_pid_.Update(yaw_err, dt);
    //printf("imu:%lf,%lf\n",pitch_imu_angle_,yaw_imu_angle_);
    //printf("data:%lf,%lf,%lf,%lf\n",pitch_err,pitch_cmd,yaw_err,yaw_cmd);
    double w_cmd=0;
    
    if(follow_mode_flag_){
        // follow mode
        double w_err = -yaw_motor_angle_;
        w_cmd = chassis_pid_.Update(w_err, dt);
    }else{
         // independent mode
        w_cmd = chassis_cmd_msg_.twist.angular.z
    }
    // publish CMD
    publishIgnGimbal(pitch_cmd,yaw_cmd);
    publishIgnChassis(chassis_cmd_msg_.twist.linear.x,
        chassis_cmd_msg_.twist.linear.y,w_cmd);
};

rcl_interfaces::msg::SetParametersResult ChassisGimbalController::parametersCb(
        const std::vector<rclcpp::Parameter> &/*parameters*/){
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // update parameters
    picth_pid_param_.p=nh_->get_parameter("pitch_pid_p").as_double();
    picth_pid_param_.i=nh_->get_parameter("pitch_pid_i").as_double();
    picth_pid_param_.d=nh_->get_parameter("pitch_pid_d").as_double();
    yaw_pid_param_.p=nh_->get_parameter("yaw_pid_p").as_double();
    yaw_pid_param_.i=nh_->get_parameter("yaw_pid_i").as_double();
    yaw_pid_param_.d =nh_->get_parameter("yaw_pid_d").as_double();
    chassis_pid_param_.p=nh_->get_parameter("chassis_pid_p").as_double();
    chassis_pid_param_.i=nh_->get_parameter("chassis_pid_i").as_double();
    chassis_pid_param_.d =nh_->get_parameter("chassis_pid_d").as_double();
    update_pid_flag_ = true;
    return result;
}

#endif //RMOSS_IGN_BASE_CHASSIS_GIMBAL_CONTROLLER_H