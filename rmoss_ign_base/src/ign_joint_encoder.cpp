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
#include "rmoss_ign_base/ign_joint_encoder.hpp"
#include <cmath>
#include <assert.h>

using namespace std;
using namespace rmoss_ign_base;


IgnJointEncoder::IgnJointEncoder(const std::shared_ptr<ignition::transport::Node> &ign_node,
               const std::string &ign_joint_state_topic){
    ign_node_=ign_node;
    ign_node_->Subscribe(ign_joint_state_topic, &IgnJointEncoder::ignJointStateCb, this);
}

void IgnJointEncoder::ignJointStateCb(const ignition::msgs::Model& msg)
{
    std::lock_guard<std::mutex> lock(msg_mut_);
    joint_state_msg_=msg;
    //init index of joints in joint_state.
    if(!init_){
        for(int i=0;i<msg.joint_size();i++){
            joint_names_.push_back(msg.joint(i).name());
            joint_idx_map_[msg.joint(i).name()]=i;
            if(msg.joint(i).name().find("pitch")!= std::string::npos){
                picth_idx_=i;
            }
            if(msg.joint(i).name().find("yaw")!= std::string::npos){
                yaw_idx_=i;
            }
        }
        init_=true;
    }
}

int IgnJointEncoder::getJointIdx(std::string &joint_name){
    if(init_){
        if(joint_idx_map_.find(joint_name)!=joint_idx_map_.end()){
            return joint_idx_map_[joint_name];
        }
    }
    return -1;
}

double IgnJointEncoder::getPos(int joint_idx){
    std::lock_guard<std::mutex> lock(msg_mut_);
    if(init_){
        assert(joint_idx >= 0 && joint_idx < (int)joint_names_.size());
        return joint_state_msg_.joint(joint_idx).axis1().position();
    }else{
        return 0;
    }
}
double IgnJointEncoder::getPos(std::string &joint_name){
    int idx=getJointIdx(joint_name);
    return getPos(idx);
}

double IgnJointEncoder::getPitch(){
    return getPos(picth_idx_);
}

double IgnJointEncoder::getYaw(){
    return getPos(yaw_idx_);
}