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
#ifndef RMOSS_IGN_BASE__IGN_ODOMETRY_HPP_
#define RMOSS_IGN_BASE__IGN_ODOMETRY_HPP_

#include <mutex>
#include <ignition/transport/Node.hh>
#include "nav_msgs/msg/odometry.hpp"

namespace rmoss_ign_base {

class IgnOdometry {
public:
    IgnOdometry(const std::shared_ptr<ignition::transport::Node> &ign_node,
               const std::string &ign_odometry_topic);
    ~IgnOdometry() {};

public:
    bool get_odometry(nav_msgs::msg::Odometry &odom);
    bool is_init(){ return init_frame_; };
    std::string get_frame_id(){ return frame_id_; };
    std::string get_child_frame_id(){ return child_frame_id_; };

private:
    void odometry_cb(const ignition::msgs::Odometry& msg);
private:
    std::shared_ptr<ignition::transport::Node> ign_node_;
    std::mutex msg_mut_;
    // sensor data
    ignition::msgs::Odometry odom_msg_;
    std::string frame_id_;
    std::string child_frame_id_;
    bool init_frame_{false};
};

}

#endif  // RMOSS_IGN_BASE__IGN_ODOMETRY_HPP_