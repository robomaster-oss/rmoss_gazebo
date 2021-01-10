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
#include "rmoss_ign_robot_base/shooter_simple_controller.hpp"

int main(int argc, char* argv[])
{
    // create ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("shooter_simple_controller");
    // variables
    std::string ros_shoot_cmd_topic,ign_shoot_cmd_topic;
    // parameters
    ros_node->declare_parameter("ros_shoot_cmd_topic","shoot_cmd");
    ros_node->declare_parameter("ign_shoot_cmd_topic");
    ros_shoot_cmd_topic = ros_node->get_parameter("ros_shoot_cmd_topic").as_string();
    ign_shoot_cmd_topic = ros_node->get_parameter("ign_shoot_cmd_topic").as_string();
    // create controller 
    auto shooter_controller = std::make_shared<rmoss_ign_robot_base::ShooterSimpleController>(ros_node,
         ros_shoot_cmd_topic, ign_shoot_cmd_topic);
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
