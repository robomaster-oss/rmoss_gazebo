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
#include "rmoss_ign_base/ign_imu.hpp"
#include <cmath>

using namespace std;
using namespace rmoss_ign_base;

double toPitch(const double &x,const double &y,const double &z,const double &w){
    // pitch (y-axis rotation)
    double pitch;
    double sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);
    return pitch;
}

double toYaw(const double &x,const double &y,const double &z,const double &w){
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    return atan2(siny_cosp, cosy_cosp);
}

IgnImu::IgnImu(const std::shared_ptr<ignition::transport::Node> &ign_node,
               const std::string &ign_gimbal_imu_topic){
    ign_node_=ign_node;
    ign_node_->Subscribe(ign_gimbal_imu_topic, &IgnImu::ignImuCb, this);
}

void IgnImu::ignImuCb(const ignition::msgs::IMU& msg)
{
    std::lock_guard<std::mutex> lock(msg_mut_);
    imu_msg_=msg;
    auto &q=imu_msg_.orientation();
    pitch_angle_ = toPitch(q.x(),q.y(),q.z(),q.w());
    yaw_angle_ = toYaw(q.x(),q.y(),q.z(),q.w());
    //continuous yaw
    continuous_yaw_angle_ = continuous_yaw_angle_ + (yaw_angle_ - last_yaw_angle_);
    if(yaw_angle_ - last_yaw_angle_>3){
        continuous_yaw_angle_ = continuous_yaw_angle_ - 3.1415926535;
    }else if(yaw_angle_ - last_yaw_angle_<-3){
        continuous_yaw_angle_ = continuous_yaw_angle_ + 3.1415926535;
    }
    last_yaw_angle_ = yaw_angle_;
}

double IgnImu::getPitch(){
    std::lock_guard<std::mutex> lock(msg_mut_);
    return pitch_angle_;
}

double IgnImu::getYaw(bool is_continuous){
    std::lock_guard<std::mutex> lock(msg_mut_);
    if(is_continuous){
        return continuous_yaw_angle_;
    }else{
        return yaw_angle_;
    }
}