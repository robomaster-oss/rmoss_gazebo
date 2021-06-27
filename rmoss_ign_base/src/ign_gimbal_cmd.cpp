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
#include "rmoss_ign_base/ign_gimbal_cmd.hpp"

using namespace std;
using namespace rmoss_ign_base;

IgnGimbalCmd::IgnGimbalCmd(const std::shared_ptr<ignition::transport::Node> &ign_node,
            const std::string &ign_pitch_cmd_topic,
            const std::string &ign_yaw_cmd_topic){
    ign_node_ =  ign_node;         
    ign_pitch_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
        ign_node_->Advertise<ignition::msgs::Double>(ign_pitch_cmd_topic));
    ign_yaw_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
        ign_node_->Advertise<ignition::msgs::Double>(ign_yaw_cmd_topic));        
}

void IgnGimbalCmd::publish(double pitch,double yaw){
    ignition::msgs::Double ign_msg;
    ign_msg.set_data(pitch);
    ign_pitch_cmd_pub_->Publish(ign_msg);
    ign_msg.set_data(yaw);
    ign_yaw_cmd_pub_->Publish(ign_msg);
}

