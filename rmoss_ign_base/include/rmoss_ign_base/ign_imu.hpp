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
#ifndef RMOSS_IGN_BASE_IGN_IMU_SUBSCRIBER_H
#define RMOSS_IGN_BASE_IGN_IMU_SUBSCRIBER_H

#include <mutex>
#include <ignition/transport/Node.hh>

namespace rmoss_ign_base {

class IgnImu {
public:
    IgnImu(const std::shared_ptr<ignition::transport::Node> &ign_node,
               const std::string &ign_gimbal_imu_topic);
    ~IgnImu() {};

public:
    double getPitch();
    double getYaw(bool is_continuous=true);

private:
    void ignImuCb(const ignition::msgs::IMU& msg);
private:
    std::shared_ptr<ignition::transport::Node> ign_node_;
    //tmp data
    double last_yaw_{0};
    std::mutex msg_mut_;
    // sensor data
    ignition::msgs::IMU imu_msg_;
    double pitch_angle_;
    double yaw_angle_;
    double last_yaw_angle_;
    double continuous_yaw_angle_;
};

}

#endif //RMOSS_IGN_BASE_IGN_IMU_SUBSCRIBER_H