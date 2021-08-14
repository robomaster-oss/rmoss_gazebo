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
#ifndef RMOSS_IGN_BASE__ODOMETRY_PUBLISHER_HPP_
#define RMOSS_IGN_BASE__ODOMETRY_PUBLISHER_HPP_


#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rmoss_ign_base/ign_odometry.hpp"

namespace rmoss_ign_base {

class OdometryPublisher {
public:
    OdometryPublisher(const rclcpp::Node::SharedPtr& nh,
        std::shared_ptr<IgnOdometry> &ign_odometry,
        const std::string& odom_topic = "odom",
        int update_rate = 50,
        bool publish_tf = true);
    ~OdometryPublisher() {};

    void set_frame_id(const std::string & frame_id);
    void set_child_frame_id(const std::string & child_frame_id);
    void set_footprint(bool use_footprint) { use_footprint_ = use_footprint;};
private:
    void timer_callback();

private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<IgnOdometry> ign_odometry_;
    // ros pub and sub
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    //
    nav_msgs::msg::Odometry odom_msg_;
    geometry_msgs::msg::TransformStamped tf_msg_;
    std::string frame_id_;
    std::string child_frame_id_;
    bool use_footprint_{false};
    bool init_frame_{false};
    bool publish_tf_{true};
};
}
#endif //RMOSS_IGN_BASE__ODOMETRY_PUBLISHER_HPP_