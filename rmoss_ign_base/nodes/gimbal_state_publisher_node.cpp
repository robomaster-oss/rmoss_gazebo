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
    // variables
    std::string ros_topic,ign_topic;
    int update_rate;
    // get parameter
    ros_node->declare_parameter("ros_topic","gimbal_state");
    ros_node->declare_parameter("ign_topic");
    ros_node->declare_parameter("update_rate", 30);
    ros_topic = ros_node->get_parameter("ros_topic").as_string();
    ign_topic = ros_node->get_parameter("ign_topic").as_string();
    update_rate = ros_node->get_parameter("update_rate").as_int();
    // create publisher
    auto gimbal_publisher = std::make_shared<rmoss_ign_base::GimbalStatePublisher>(ros_node,
        ros_topic, ign_topic, 0, 1 ,update_rate);
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
