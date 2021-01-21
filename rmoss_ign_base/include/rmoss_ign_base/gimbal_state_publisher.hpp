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

#include "rmoss_interfaces/msg/gimbal_cmd.hpp"
#include <ignition/transport/Node.hh>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

namespace rmoss_ign_base {

class GimbalStatePublisher {
public:
    GimbalStatePublisher(const rclcpp::Node::SharedPtr& nh,
        const std::string& ros_topic,
        const std::string& ign_topic,
        const int& ign_pitch_idx,
        const int& ign_yaw_idx,
        const unsigned int& update_rate);
    ~GimbalStatePublisher() {};

private:
    void gimbalStateTimerCb();
    void ignJointStateCb(const ignition::msgs::Model& msg);

private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<ignition::transport::Node> ign_node_;
    // ros pub and sub
    rclcpp::Publisher<rmoss_interfaces::msg::Gimbal>::SharedPtr ros_gimbal_state_pub_;
    rclcpp::TimerBase::SharedPtr gimbal_state_timer_;
    //data
    ignition::msgs::Model current_joint_msg_;
    std::mutex current_joint_msg_mut_;
    int ign_pitch_idx_;
    int ign_yaw_idx_;
};
}
#endif //RMOSS_IGN_BASE_GIMBAL_STATE_PUBLISHER_H