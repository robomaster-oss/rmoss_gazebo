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
        const std::string& ros_gimbal_state_topic,
        std::shared_ptr<IgnImu> &ign_gimbal_imu,
        unsigned int update_rate)
{
    // ROS and Ignition node
    nh_ = nh;
    ign_gimbal_imu_ = ign_gimbal_imu;
    //create ros pub and timer
    ros_gimbal_state_pub_ = nh_->create_publisher<rmoss_interfaces::msg::Gimbal>(
        ros_gimbal_state_topic, 10);
    auto period = std::chrono::microseconds(1000000 / update_rate);
    gimbal_state_timer_ = nh_->create_wall_timer(period, 
        std::bind(&GimbalStatePublisher::gimbalStateTimerCb, this));
}

void GimbalStatePublisher::gimbalStateTimerCb()
{

    rmoss_interfaces::msg::Gimbal gimbal_state;
    gimbal_state.pitch = ign_gimbal_imu_->getPitch();
    gimbal_state.yaw = ign_gimbal_imu_->getYaw();
    ros_gimbal_state_pub_->publish(gimbal_state);
}
