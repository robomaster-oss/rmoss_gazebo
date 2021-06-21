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

double toPitch(const double &x,const double &y,const double &z,const double &w){
    // pitch (y-axis rotation)
    double pitch;
    double sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);
    return pitch;
}

double toYaw(const double &x,const double &y,const double &z,const double &w){
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    return atan2(siny_cosp, cosy_cosp);
}

ChassisGimbalControllerBase::ChassisGimbalControllerBase(const rclcpp::Node::SharedPtr& nh){
    nh_=nh;
    ign_node_ = std::make_shared<ignition::transport::Node>();
    auto ros_gimbal_state_topic = nh_->get_parameter("ros_gimbal_state_topic").as_string();
    auto ros_chassis_cmd_topic = nh_->get_parameter("ros_chassis_cmd_topic").as_string();
    auto ros_gimbal_cmd_topic = nh_->get_parameter("ros_gimbal_cmd_topic").as_string();
    auto ign_chassis_cmd_topic = nh_->get_parameter("ign_chassis_cmd_topic").as_string();
    auto ign_pitch_cmd_topic = nh_->get_parameter("ign_pitch_cmd_topic").as_string();
    auto ign_yaw_cmd_topic = nh_->get_parameter("ign_yaw_cmd_topic").as_string();
    auto ign_joint_state_topic = nh_->get_parameter("ign_joint_state_topic").as_string();
    auto ign_chassis_imu_topic = nh_->get_parameter("ign_chassis_imu_topic").as_string();
    auto ign_gimbal_imu_topic = nh_->get_parameter("ign_gimbal_imu_topic").as_string();
    // create ros pub
    ros_gimbal_state_pub_ = nh_->create_publisher<rmoss_interfaces::msg::Gimbal>(ros_gimbal_state_topic, 10);
    // create ros sub
    ros_chassis_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::ChassisCmd>(ros_chassis_cmd_topic,
        10, std::bind(&ChassisGimbalControllerBase::chassisCb, this, std::placeholders::_1));
    ros_gimbal_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::GimbalCmd>(ros_gimbal_cmd_topic,
        10, std::bind(&ChassisGimbalControllerBase::gimbalCb, this, std::placeholders::_1));
    // create ign pub
    ign_chassis_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
        ign_node_->Advertise<ignition::msgs::Twist>(ign_chassis_cmd_topic));
    ign_gimbal_pitch_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
        ign_node_->Advertise<ignition::msgs::Double>(ign_pitch_cmd_topic));
    ign_gimbal_yaw_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
        ign_node_->Advertise<ignition::msgs::Double>(ign_yaw_cmd_topic));
    // create ign sub
    ign_node_->Subscribe(ign_joint_state_topic, &ChassisGimbalControllerBase::ignJointStateCb, this);
    ign_node_->Subscribe(ign_gimbal_imu_topic, &ChassisGimbalControllerBase::ignGimbalImuCb, this);
}


void ChassisGimbalControllerBase::chassisCb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(msg_mut_);
    chassis_cmd_msg_ = *msg;
}

void ChassisGimbalControllerBase::gimbalCb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(msg_mut_);
    gimbal_cmd_msg_ = *msg;
}

void ChassisGimbalControllerBase::ignJointStateCb(const ignition::msgs::Model& msg)
{
    std::lock_guard<std::mutex> lock(msg_mut_);
    if(msg.joint_size()==2){
        pitch_motor_angle_ = msg.joint(1).axis1().position();
        yaw_motor_angle_ = msg.joint(0).axis1().position();
    }
}


void ChassisGimbalControllerBase::ignGimbalImuCb(const ignition::msgs::IMU& msg)
{
    std::lock_guard<std::mutex> lock(msg_mut_);
    auto q=msg.orientation();
    // pitch
    pitch_imu_angle_ = toPitch(q.x(),q.y(),q.z(),q.w());
    // yaw
    double current_yaw = toYaw(q.x(),q.y(),q.z(),q.w());
    yaw_imu_angle_ = yaw_imu_angle_ + (current_yaw - last_yaw_);
    if(current_yaw - last_yaw_>3){
        yaw_imu_angle_ = yaw_imu_angle_ - 3.1415926535;
    }else if(current_yaw - last_yaw_<-3){
        yaw_imu_angle_ = yaw_imu_angle_ + 3.1415926535;
    }
    last_yaw_ = current_yaw;
    
}

void ChassisGimbalControllerBase::publishIgnChassis(double v_x,double v_y,double v_w){
    ignition::msgs::Twist ign_msg;
    ign_msg.mutable_linear()->set_x(v_x);
    ign_msg.mutable_linear()->set_y(v_y);
    ign_msg.mutable_angular()->set_z(v_w);
    ign_chassis_cmd_pub_->Publish(ign_msg);
}
void ChassisGimbalControllerBase::publishIgnGimbal(double v_pitch,double v_yaw){
    ignition::msgs::Double ign_msg;
    ign_msg.set_data(v_pitch);
    ign_gimbal_pitch_cmd_pub_->Publish(ign_msg);
    ign_msg.set_data(v_yaw);
    ign_gimbal_yaw_cmd_pub_->Publish(ign_msg);
}