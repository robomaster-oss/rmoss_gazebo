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
#include "rmoss_ign_base/simple_shooter_controller.hpp"

using namespace std;
using namespace rmoss_ign_base;

SimpleShooterController::SimpleShooterController(const rclcpp::Node::SharedPtr& nh,
    const std::string& ros_cmd_topic,
    const std::string& ign_cmd_topic)
{
    // ROS and Ignition node
    nh_ = nh;
    ign_node_ = std::make_shared<ignition::transport::Node>();
    //create ros pub and sub
    ros_shoot_cmd_sub_ = nh_->create_subscription<rmoss_interfaces::msg::ShootCmd>(ros_cmd_topic,
        10, std::bind(&SimpleShooterController::shootCb, this, std::placeholders::_1));
    //create ignition pub
    ign_shoot_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
        ign_node_->Advertise<ignition::msgs::Int32>(ign_cmd_topic));
}

void SimpleShooterController::shootCb(const rmoss_interfaces::msg::ShootCmd::SharedPtr msg)
{
    ignition::msgs::Int32 ign_msg;
    ign_msg.set_data(msg->projectile_num);
    ign_shoot_cmd_pub_->Publish(ign_msg);
    //std::cout << " shootCb,shoot num:" << (int)(msg->projectile_num) << std::endl;
}