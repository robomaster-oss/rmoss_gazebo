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
#ifndef RMOSS_IGN_BASE_CHASSIS_GIMBAL_CONTROLLER_BASE_H
#define RMOSS_IGN_BASE_CHASSIS_GIMBAL_CONTROLLER_BASE_H

#include <mutex>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <ignition/transport/Node.hh>

#include "rmoss_interfaces/msg/chassis_cmd.hpp"
#include "rmoss_interfaces/msg/gimbal_cmd.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"

namespace rmoss_ign_base {

class ChassisGimbalControllerBase {
public:
    ChassisGimbalControllerBase(const rclcpp::Node::SharedPtr& nh);
    ~ChassisGimbalControllerBase() {};
protected:
    void publishIgnChassis(double v_x,double v_y,double v_w);
    void publishIgnGimbal(double v_pitch,double v_yaw);
private:
    void chassisCb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg);
    void gimbalCb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg);
    void ignJointStateCb(const ignition::msgs::Model& msg);
    void ignGimbalImuCb(const ignition::msgs::IMU& msg);
    void ignChassisImuCb(const ignition::msgs::IMU& msg);
private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<ignition::transport::Node> ign_node_;
    // ros pub
    rclcpp::Publisher<rmoss_interfaces::msg::Gimbal>::SharedPtr ros_gimbal_state_pub_;
    // ros sub
    rclcpp::Subscription<rmoss_interfaces::msg::ChassisCmd>::SharedPtr ros_chassis_cmd_sub_;
    rclcpp::Subscription<rmoss_interfaces::msg::GimbalCmd>::SharedPtr ros_gimbal_cmd_sub_;
    // ign pub
    std::unique_ptr<ignition::transport::Node::Publisher> ign_chassis_cmd_pub_;
    std::unique_ptr<ignition::transport::Node::Publisher> ign_gimbal_pitch_cmd_pub_;
    std::unique_ptr<ignition::transport::Node::Publisher> ign_gimbal_yaw_cmd_pub_;
    //tmp data
    double last_yaw_{0};
protected:
    std::mutex msg_mut_;
    // target data
    rmoss_interfaces::msg::ChassisCmd chassis_cmd_msg_;
    rmoss_interfaces::msg::GimbalCmd gimbal_cmd_msg_;
    // sensor data
    double pitch_motor_angle_{0};
    double yaw_motor_angle_{0};
    double pitch_imu_angle_{0};
    double yaw_imu_angle_{0};
};

}

#endif //RMOSS_IGN_BASE_CHASSIS_GIMBAL_CONTROLLER_BASE_H