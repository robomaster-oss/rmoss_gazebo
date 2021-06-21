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
#include "rmoss_ign_base/simple_chassis_controller.hpp"

using namespace std;
using namespace rmoss_ign_base;

SimpleChassisController::SimpleChassisController(const rclcpp::Node::SharedPtr& nh,
    const std::string& ros_cmd_topic,
    const std::string& ign_cmd_topic)
{
    // ROS and Ignition node
    nh_ = nh;
    ign_node_ = std::make_shared<ignition::transport::Node>();
    //create ros pub and sub
    ros_chassis_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::ChassisCmd>(ros_cmd_topic,
        10, std::bind(&SimpleChassisController::chassisCb, this, std::placeholders::_1));
    //create ignition pub
    ign_chassis_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
        ign_node_->Advertise<ignition::msgs::Twist>(ign_cmd_topic));
}

void SimpleChassisController::chassisCb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg)
{
    ignition::msgs::Twist ign_msg;
    ign_msg.mutable_linear()->set_x(msg->twist.linear.x);
    ign_msg.mutable_linear()->set_y(msg->twist.linear.y);
    ign_msg.mutable_angular()->set_z(msg->twist.angular.z);
    ign_chassis_cmd_pub_->Publish(ign_msg);
}