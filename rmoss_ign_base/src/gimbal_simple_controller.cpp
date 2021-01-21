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
#include "rmoss_ign_base/gimbal_simple_controller.hpp"

using namespace std;
using namespace rmoss_ign_base;

GimbalSimpleController::GimbalSimpleController(const rclcpp::Node::SharedPtr& nh,
    const std::string& ros_cmd_topic,
    const std::string& ign_pitch_topic,
    const std::string& ign_yaw_topic)
{
    // ROS and Ignition node
    nh_ = nh;
    ign_node_ = std::make_shared<ignition::transport::Node>();
    //create ros pub and sub
    ros_gimbal_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::GimbalCmd>(ros_cmd_topic,
        10, std::bind(&GimbalSimpleController::gimbalCb, this, std::placeholders::_1));
    //create ignition pub
    ign_pitch_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
        ign_node_->Advertise<ignition::msgs::Double>(ign_pitch_topic));
    ign_yaw_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
        ign_node_->Advertise<ignition::msgs::Double>(ign_yaw_topic));
}

void GimbalSimpleController::gimbalCb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg)
{
    ignition::msgs::Double ign_msg;
    ign_msg.set_data(msg->position.pitch);
    ign_pitch_pub_->Publish(ign_msg);
    ign_msg.set_data(msg->position.yaw);
    ign_yaw_pub_->Publish(ign_msg);
}