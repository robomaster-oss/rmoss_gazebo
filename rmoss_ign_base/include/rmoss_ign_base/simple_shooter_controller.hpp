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
#ifndef RMOSS_IGN_BASE_SIMPLE_SHOOTER_CONTROLLER_H
#define RMOSS_IGN_BASE_SIMPLE_SHOOTER_CONTROLLER_H

#include <mutex>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include "rmoss_interfaces/msg/shoot_cmd.hpp"

namespace rmoss_ign_base {

class SimpleShooterController {
public:
    SimpleShooterController(const rclcpp::Node::SharedPtr& nh,
        const std::string& ros_cmd_topic,
        const std::string& ign_cmd_topic);
    ~SimpleShooterController() {};

private:
    void shootCb(const rmoss_interfaces::msg::ShootCmd::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<ignition::transport::Node> ign_node_;
    // ros pub and sub
    rclcpp::Subscription<rmoss_interfaces::msg::ShootCmd>::SharedPtr ros_shoot_cmd_sub_;
    // ign pub and sub
    std::unique_ptr<ignition::transport::Node::Publisher> ign_shoot_cmd_pub_;
};
}
#endif //RMOSS_IGN_BASE_SIMPLE_SHOOTER_CONTROLLER_H