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
#ifndef RMOSS_IGN_BASE_GIMBAL_STATE_PUBLISHER_H
#define RMOSS_IGN_BASE_GIMBAL_STATE_PUBLISHER_H


#include <rclcpp/rclcpp.hpp>
#include <rmoss_interfaces/msg/gimbal_cmd.hpp>

#include "rmoss_ign_base/ign_imu.hpp"

namespace rmoss_ign_base {

class GimbalStatePublisher {
public:
    GimbalStatePublisher(const rclcpp::Node::SharedPtr& nh,
        const std::string& ros_gimbal_state_topic,
        std::shared_ptr<IgnImu> &ign_gimbal_imu,
        unsigned int update_rate);
    ~GimbalStatePublisher() {};

private:
    void gimbalStateTimerCb();

private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<IgnImu> ign_gimbal_imu_;
    // ros pub and sub
    rclcpp::Publisher<rmoss_interfaces::msg::Gimbal>::SharedPtr ros_gimbal_state_pub_;
    rclcpp::TimerBase::SharedPtr gimbal_state_timer_;
};
}
#endif //RMOSS_IGN_BASE_GIMBAL_STATE_PUBLISHER_H