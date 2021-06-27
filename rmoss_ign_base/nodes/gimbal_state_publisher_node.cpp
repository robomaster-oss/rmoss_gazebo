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
#include <rclcpp/rclcpp.hpp>
#include "rmoss_ign_base/gimbal_state_publisher.hpp"

int main(int argc, char* argv[])
{
    // create ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("gimbal_state_publisher");
    auto ign_node = std::make_shared<ignition::transport::Node>();
    // get parameter
    ros_node->declare_parameter("ign_gimbal_imu_topic");
    ros_node->declare_parameter("update_rate", 30);
    auto ros_gimbal_state_topic = ros_node->get_parameter("ros_topic").as_string();
    auto ign_gimbal_imu_topic = ros_node->get_parameter("ign_gimbal_imu_topic").as_string();
    auto update_rate = ros_node->get_parameter("update_rate").as_int();
    // ign gimbal sensor (imu)
    auto ign_gimbal_imu = std::make_shared<rmoss_ign_base::IgnImu>(ign_node,ign_gimbal_imu_topic);
    // create publisher
    auto gimbal_publisher = std::make_shared<rmoss_ign_base::GimbalStatePublisher>(ros_node,
        "gimbal_state",ign_gimbal_imu ,update_rate);
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
