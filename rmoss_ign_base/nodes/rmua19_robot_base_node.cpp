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
#include "rmoss_ign_base/simple_gimbal_controller.hpp"
#include "rmoss_ign_base/simple_shooter_controller.hpp"
#include "rmoss_ign_base/gimbal_state_publisher.hpp"


int main(int argc, char* argv[])
{
    // create ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("robot_base");
    // variables
    std::string world_name,robot_name;
    // parameters
    ros_node->declare_parameter("world_name");
    world_name = ros_node->get_parameter("world_name").as_string();
    ros_node->declare_parameter("robot_name");
    robot_name = ros_node->get_parameter("robot_name").as_string();
    // topic string 
    std::string ign_chassis_cmd_topic = "/"+robot_name+"/cmd_vel";
    std::string ign_gimbal_pitch_cmd_topic = "/model/"+robot_name+"/joint/gimbal_pitch_joint/0/cmd_pos";
    std::string ign_gimbal_yaw_cmd_topic = "/model/"+robot_name+"/joint/gimbal_yaw_joint/0/cmd_pos";
    std::string ign_joint_state_topic = "/world/"+world_name+"/model/"+robot_name+"/joint_state";
    std::string ign_shooter_cmd_topic = "/"+robot_name+"/small_shooter/shoot";
    // create controller 
    auto chassis_controller = std::make_shared<rmoss_ign_base::SimpleChassisController>(
        ros_node,"chassis_cmd", ign_chassis_cmd_topic);
    auto gimbal_controller = std::make_shared<rmoss_ign_base::SimpleGimbalController>(
        ros_node,"gimbal_cmd", ign_gimbal_pitch_cmd_topic, ign_gimbal_yaw_cmd_topic);
    auto shooter_controller = std::make_shared<rmoss_ign_base::SimpleShooterController>(
        ros_node,"shoot_cmd",ign_shooter_cmd_topic);
    // create gimbal publisher
    auto gimbal_publisher = std::make_shared<rmoss_ign_base::GimbalStatePublisher>(
        ros_node,"gimbal_state", ign_joint_state_topic,
        0, 1 , 50);
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
