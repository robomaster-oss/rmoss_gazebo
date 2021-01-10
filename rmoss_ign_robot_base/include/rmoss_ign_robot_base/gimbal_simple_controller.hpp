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
#ifndef RMOSS_IGN_ROBOT_BASE_GIMBAL_SIMPLE_CONTROLLER_H
#define RMOSS_IGN_ROBOT_BASE_GIMBAL_SIMPLE_CONTROLLER_H

#include <mutex>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include "rmoss_interfaces/msg/gimbal_cmd.hpp"

namespace rmoss_ign_robot_base {

class GimbalSimpleController {
public:
    GimbalSimpleController(const rclcpp::Node::SharedPtr& nh,
    const std::string& ros_cmd_topic,
    const std::string& ign_pitch_topic,
    const std::string& ign_yaw_topic);
    ~GimbalSimpleController() {};

private:
    void gimbalCb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<ignition::transport::Node> ign_node_;
    // ros pub and sub
    rclcpp::Subscription<rmoss_interfaces::msg::GimbalCmd>::SharedPtr ros_gimbal_cmd_sub_;
    std::unique_ptr<ignition::transport::Node::Publisher> ign_pitch_pub_;
    std::unique_ptr<ignition::transport::Node::Publisher> ign_yaw_pub_;
};
}
#endif //RMOSS_IGN_ROBOT_BASE_GIMBAL_SIMPLE_CONTROLLER_H