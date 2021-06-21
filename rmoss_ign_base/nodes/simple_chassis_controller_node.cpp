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
#include "rmoss_ign_base/simple_chassis_controller.hpp"

int main(int argc, char* argv[])
{
    // create ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("simple_chassis_controller");
    // variables
    std::string ros_chassis_cmd_topic,ign_chassis_cmd_topic;
    // parameters
    ros_node->declare_parameter("ros_chassis_cmd_topic","chassis_cmd");
    ros_node->declare_parameter("ign_chassis_cmd_topic");
    ros_chassis_cmd_topic = ros_node->get_parameter("ros_chassis_cmd_topic").as_string();
    ign_chassis_cmd_topic = ros_node->get_parameter("ign_chassis_cmd_topic").as_string();
    // create controller 
    auto chassis_controller = std::make_shared<rmoss_ign_base::SimpleChassisController>(ros_node,
         ros_chassis_cmd_topic, ign_chassis_cmd_topic);
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
