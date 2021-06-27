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
#include "rmoss_ign_base/ign_chassis_cmd.hpp"

using namespace std;
using namespace rmoss_ign_base;

IgnChassisCmd::IgnChassisCmd(const std::shared_ptr<ignition::transport::Node> &ign_node,
            const std::string &ign_chassis_cmd_topic){
    ign_node_ =  ign_node;         
    ign_chassis_cmd_pub_ = std::make_unique<ignition::transport::Node::Publisher>(
        ign_node_->Advertise<ignition::msgs::Twist>(ign_chassis_cmd_topic));      
}

void IgnChassisCmd::publish(double v_x,double v_y,double v_w){
    ignition::msgs::Twist ign_msg;
    ign_msg.mutable_linear()->set_x(v_x);
    ign_msg.mutable_linear()->set_y(v_y);
    ign_msg.mutable_angular()->set_z(v_w);
    ign_chassis_cmd_pub_->Publish(ign_msg);
}

