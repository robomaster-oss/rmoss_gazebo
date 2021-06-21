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
#include "rmoss_ign_base/simple_gimbal_controller.hpp"

using namespace std;
using namespace rmoss_ign_base;

SimpleGimbalController::SimpleGimbalController(const rclcpp::Node::SharedPtr& nh,
    const std::string& ros_cmd_topic,
    const std::string& ign_pitch_cmd_topic,
    const std::string& ign_yaw_cmd_topic,
    bool position_control)
{
    // ROS and Ignition node
    nh_ = nh;
    ign_node_ = std::make_shared<ignition::transport::Node>();
    is_position_controller_=position_control;
    //create ros pub and sub
    ros_gimbal_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::GimbalCmd>(ros_cmd_topic,
        10, std::bind(&SimpleGimbalController::gimbalCb, this, std::placeholders::_1));
    //create ignition pub
    ign_pitch_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
        ign_node_->Advertise<ignition::msgs::Double>(ign_pitch_cmd_topic));
    ign_yaw_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
        ign_node_->Advertise<ignition::msgs::Double>(ign_yaw_cmd_topic));
}

void SimpleGimbalController::gimbalCb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg)
{
    ignition::msgs::Double ign_msg;
    if(is_position_controller_){
        ign_msg.set_data(msg->position.pitch);
        ign_pitch_pub_->Publish(ign_msg);
        ign_msg.set_data(msg->position.yaw);
        ign_yaw_pub_->Publish(ign_msg);
    }else{
        ign_msg.set_data(msg->velocity.pitch);
        ign_pitch_pub_->Publish(ign_msg);
        ign_msg.set_data(msg->velocity.yaw);
        ign_yaw_pub_->Publish(ign_msg);  
    }
}