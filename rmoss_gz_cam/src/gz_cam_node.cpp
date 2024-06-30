// Copyright 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rmoss_gz_cam/gz_cam_node.hpp"

#include <memory>
#include <vector>
#include <string>
#include <future>

#include "ros_gz_bridge/convert.hpp"

using namespace std::chrono_literals;

namespace rmoss_gz_cam
{
GzCamNode::GzCamNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("gz_cam", options);
  gz_node_ = std::make_shared<ignition::transport::Node>();
  // declare parameters
  node_->declare_parameter("gz_camera_image_topic", "");
  node_->declare_parameter("gz_camera_info_topic", "");
  node_->declare_parameter("camera_name", camera_name_);
  node_->declare_parameter("frame_id", camera_frame_id_);
  node_->declare_parameter("autostart", run_flag_);
  node_->declare_parameter("use_sensor_data_qos", use_qos_profile_sensor_data_);
  node_->declare_parameter(
    "use_image_transport_camera_publisher", use_image_transport_camera_publisher_);
  // get parameters
  auto gz_camera_image_topic = node_->get_parameter("gz_camera_image_topic").as_string();
  auto gz_camera_info_topic = node_->get_parameter("gz_camera_info_topic").as_string();
  node_->get_parameter("camera_name", camera_name_);
  node_->get_parameter("frame_id", camera_frame_id_);
  node_->get_parameter("autostart", run_flag_);
  node_->get_parameter("use_sensor_data_qos", use_qos_profile_sensor_data_);
  node_->get_parameter(
    "use_image_transport_camera_publisher", use_image_transport_camera_publisher_);
  // get camera info form gazebo
  if (gz_camera_info_topic != "") {
    // get camera info automatically
    std::promise<bool> prom;
    std::future<bool> future_result = prom.get_future();
    std::function<void(const ignition::msgs::CameraInfo & msg)> camera_info_cb =
      [&](const ignition::msgs::CameraInfo & msg) {
        ros_gz_bridge::convert_gz_to_ros(msg, cam_info_);
        cam_info_valid_ = true;
        prom.set_value(true);
      };
    gz_node_->Subscribe(gz_camera_info_topic, camera_info_cb);
    if (future_result.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_FATAL(node_->get_logger(), "failed to get camera info");
    }
    gz_node_->Unsubscribe(gz_camera_info_topic);
  }
  // create image_transport
  if (!use_image_transport_camera_publisher_) {
    img_pub_ = std::make_shared<image_transport::Publisher>(
      image_transport::create_publisher(
        node_.get(), camera_name_ + "/image_raw",
        use_qos_profile_sensor_data_ ? rmw_qos_profile_sensor_data : rmw_qos_profile_default));
  } else {
    cam_pub_ =
      std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(
        node_.get(), camera_name_ + "/image_raw",
        use_qos_profile_sensor_data_ ? rmw_qos_profile_sensor_data : rmw_qos_profile_default));
  }
  // create gazebo image subscriber
  auto ret = gz_node_->Subscribe(gz_camera_image_topic, &GzCamNode::gz_image_cb, this);
  if (!ret) {
    RCLCPP_FATAL(node_->get_logger(), "failed to create ignition subscriber");
    return;
  }
  // create GetCameraInfo service
  using namespace std::placeholders;
  get_camera_info_srv_ = node_->create_service<rmoss_interfaces::srv::GetCameraInfo>(
    camera_name_ + "/get_camera_info", std::bind(&GzCamNode::get_camera_info_cb, this, _1, _2));
  RCLCPP_INFO(node_->get_logger(), "init successfully!");
}

void GzCamNode::gz_image_cb(const ignition::msgs::Image & msg)
{
  if (!run_flag_) {
    return;
  }
  sensor_msgs::msg::Image ros_msg;
  ros_gz_bridge::convert_gz_to_ros(msg, ros_msg);
  rclcpp::Time stamp = node_->now();
  ros_msg.header.stamp = stamp;
  ros_msg.header.frame_id = camera_frame_id_;
  // publish image msg
  if (this->cam_pub_->getNumSubscribers() > 0) {
    if (!use_image_transport_camera_publisher_) {
      img_pub_->publish(ros_msg);
    } else {
      cam_info_.header.stamp = stamp;
      cam_info_.header.frame_id = camera_frame_id_;
      cam_pub_->publish(ros_msg, cam_info_);
    }
  }
}

void GzCamNode::get_camera_info_cb(
  const rmoss_interfaces::srv::GetCameraInfo::Request::SharedPtr request,
  rmoss_interfaces::srv::GetCameraInfo::Response::SharedPtr response)
{
  (void)request;
  if (cam_info_valid_) {
    response->camera_info = cam_info_;
    response->success = true;
  } else {
    response->success = false;
  }
}

}  // namespace rmoss_gz_cam

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmoss_gz_cam::GzCamNode)
