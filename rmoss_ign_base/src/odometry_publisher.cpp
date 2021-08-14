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
#include "rmoss_ign_base/odometry_publisher.hpp"

namespace rmoss_ign_base{

OdometryPublisher::OdometryPublisher(const rclcpp::Node::SharedPtr& nh,
        std::shared_ptr<IgnOdometry> &ign_odometry,
        const std::string& odom_topic, int update_rate, bool publish_tf)
    :publish_tf_(publish_tf)
{
    // ROS and Ignition node
    nh_ = nh;
    ign_odometry_ = ign_odometry;
    //create ros pub and timer
    odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
    auto period = std::chrono::microseconds(1000000 / update_rate);
    odom_timer_ = nh_->create_wall_timer(period, 
        std::bind(&OdometryPublisher::timer_callback, this));
}

void OdometryPublisher::set_frame_id(const std::string & frame_id)
{ 
    odom_msg_.header.frame_id = frame_id;
    tf_msg_.header.frame_id = frame_id;
}

void OdometryPublisher::set_child_frame_id(const std::string & child_frame_id)
{ 
    odom_msg_.child_frame_id = child_frame_id;
    tf_msg_.child_frame_id = child_frame_id;
}

void OdometryPublisher::timer_callback()
{
    ign_odometry_->get_odometry(odom_msg_);
    if(odom_msg_.header.frame_id.empty()){
        odom_msg_.header.frame_id = ign_odometry_->get_frame_id();
        tf_msg_.header.frame_id = odom_msg_.header.frame_id;
    }
    if(odom_msg_.child_frame_id.empty()){
        odom_msg_.child_frame_id = ign_odometry_->get_child_frame_id();
        tf_msg_.child_frame_id = odom_msg_.child_frame_id;
    }
    if(use_footprint_){
        odom_msg_.pose.pose.position.z = 0;
    }
    auto time = nh_->get_clock()->now();
    odom_msg_.header.stamp = time;
    tf_msg_.header.stamp = time;
    tf_msg_.transform.translation.x = odom_msg_.pose.pose.position.x;
    tf_msg_.transform.translation.y = odom_msg_.pose.pose.position.y;
    tf_msg_.transform.translation.z = odom_msg_.pose.pose.position.z;
    tf_msg_.transform.rotation = odom_msg_.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg_);
    odom_pub_->publish(odom_msg_);
}

}
