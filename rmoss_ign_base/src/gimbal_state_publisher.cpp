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
#include "rmoss_ign_base/gimbal_state_publisher.hpp"

using namespace std;
using namespace rmoss_ign_base;

GimbalStatePublisher::GimbalStatePublisher(const rclcpp::Node::SharedPtr& nh,
    const std::string& ros_topic,
    const std::string& ign_topic,
    const int& ign_pitch_idx,
    const int& ign_yaw_idx,
    const unsigned int& update_rate)
{
    // ROS and Ignition node
    nh_ = nh;
    ign_node_ = std::make_shared<ignition::transport::Node>();
    ign_pitch_idx_ = ign_pitch_idx;
    ign_yaw_idx_ = ign_yaw_idx;
    //create ros pub and timer
    ros_gimbal_state_pub_ = nh_->create_publisher<rmoss_interfaces::msg::Gimbal>(ros_topic, 10);
    auto period = std::chrono::microseconds(1000000 / update_rate);
    gimbal_state_timer_ = nh_->create_wall_timer(period, std::bind(&GimbalStatePublisher::gimbalStateTimerCb, this));
    //create ignition sub
    ign_node_->Subscribe(ign_topic, &GimbalStatePublisher::ignJointStateCb, this);
}

void GimbalStatePublisher::gimbalStateTimerCb()
{
    std::lock_guard<std::mutex> lock(current_joint_msg_mut_);
    rmoss_interfaces::msg::Gimbal gimbal_state;
    if (current_joint_msg_.joint_size() > ign_pitch_idx_ && current_joint_msg_.joint_size() > ign_yaw_idx_) {
        gimbal_state.pitch = current_joint_msg_.joint(ign_pitch_idx_).axis1().position();
        gimbal_state.yaw = current_joint_msg_.joint(ign_yaw_idx_).axis1().position();
    }
    ros_gimbal_state_pub_->publish(gimbal_state);
}

void GimbalStatePublisher::ignJointStateCb(const ignition::msgs::Model& msg)
{
    std::lock_guard<std::mutex> lock(current_joint_msg_mut_);
    current_joint_msg_ = msg;
}