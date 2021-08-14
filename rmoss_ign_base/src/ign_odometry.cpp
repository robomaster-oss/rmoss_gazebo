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
#include "rmoss_ign_base/ign_odometry.hpp"
#include <cmath>

namespace rmoss_ign_base{

IgnOdometry::IgnOdometry(const std::shared_ptr<ignition::transport::Node> &ign_node,
               const std::string &ign_odometry_topic){
    ign_node_=ign_node;
    ign_node_->Subscribe(ign_odometry_topic, &IgnOdometry::odometry_cb, this);
}

void IgnOdometry::odometry_cb(const ignition::msgs::Odometry& msg)
{
    std::lock_guard<std::mutex> lock(msg_mut_);
    if(!init_frame_){
        for (auto i = 0; i < msg.header().data_size(); ++i) {
            auto aPair = msg.header().data(i);
            if (aPair.key() == "frame_id" && aPair.value_size() > 0) {
               frame_id_ = aPair.value(0);
            }else if(aPair.key() == "child_frame_id" && aPair.value_size() > 0){
                child_frame_id_  = aPair.value(0);
            }
        }
        init_frame_ = true;
    }
    odom_msg_=msg;
}

bool IgnOdometry::get_odometry(nav_msgs::msg::Odometry &odom){
    std::lock_guard<std::mutex> lock(msg_mut_);
    if(!init_frame_){
        return false;
    }
    odom.pose.pose.position.x = odom_msg_.pose().position().x();
    odom.pose.pose.position.y = odom_msg_.pose().position().y();
    odom.pose.pose.position.z = odom_msg_.pose().position().z();
    odom.pose.pose.orientation.x = odom_msg_.pose().orientation().x();
    odom.pose.pose.orientation.y = odom_msg_.pose().orientation().y();
    odom.pose.pose.orientation.z = odom_msg_.pose().orientation().z();
    odom.pose.pose.orientation.w = odom_msg_.pose().orientation().w();
    odom.twist.twist.linear.x = odom_msg_.twist().linear().x();
    odom.twist.twist.linear.y = odom_msg_.twist().linear().y();
    odom.twist.twist.linear.z = odom_msg_.twist().linear().z();
    odom.twist.twist.angular.x = odom_msg_.twist().angular().x();
    odom.twist.twist.angular.y = odom_msg_.twist().angular().y();
    odom.twist.twist.angular.z = odom_msg_.twist().angular().z();
    return true;
}

}