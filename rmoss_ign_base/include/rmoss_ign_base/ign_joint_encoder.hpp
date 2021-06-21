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
#ifndef RMOSS_IGN_BASE_IGN_JOINT_ENCODER_H
#define RMOSS_IGN_BASE_IGN_JOINT_ENCODER_H

#include <mutex>
#include <set>
#include <vector>
#include <ignition/transport/Node.hh>

namespace rmoss_ign_base {

class IgnJointEncoder {
public:
    IgnJointEncoder(const std::shared_ptr<ignition::transport::Node> &ign_node,
               const std::string &ign_joint_state_topic);
    ~IgnJointEncoder() {};

public:
    // get position of joint
    int getJointIdx(std::string &joint_name);
    double getPos(int joint_idx);
    double getPos(std::string &joint_name);
    // extra API for pitch and yaw
    void setPitchJointName();
    void setYawJointName();
    double getPitch();
    double getYaw();

private:
    void ignJointStateCb(const ignition::msgs::Model& msg);
private:
    std::shared_ptr<ignition::transport::Node> ign_node_;
    // sensor data
    ignition::msgs::Model joint_state_msg_;
    std::mutex msg_mut_;
    // info
    std::vector<std::string> joint_names_;
    std::map<std::string,int> joint_idx_map_;
    bool init_{false};
    // extra 
    int yaw_idx_{-1};
    int picth_idx_{-1};
};

}

#endif //RMOSS_IGN_BASE_IGN_JOINT_ENCODER_H