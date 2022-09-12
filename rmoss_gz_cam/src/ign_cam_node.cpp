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

#include "rmoss_ign_cam/ign_cam_node.hpp"

#include <memory>
#include <vector>
#include <string>
#include <future>

namespace rmoss_ign_cam
{
IgnCamNode::IgnCamNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("ign_cam", options);
  ign_node_ = std::make_shared<ignition::transport::Node>();
  // declare parameters
  node_->declare_parameter("ign_camera_image_topic", "");
  node_->declare_parameter("ign_camera_info_topic", "");
  // get parameters
  auto ign_camera_image_topic = node_->get_parameter("ign_camera_image_topic").as_string();
  auto ign_camera_info_topic = node_->get_parameter("ign_camera_info_topic").as_string();
  int height = 640;
  int width = 480;
  if (ign_camera_info_topic != "") {
    // get camera info automatically
    std::vector<double> camera_k_{0, 0, 0, 0, 0, 0, 0, 0, 0};  // 3*3=9
    std::vector<double> camera_d_{0, 0, 0, 0, 0};
    std::promise<bool> prom;
    std::future<bool> future_result = prom.get_future();
    std::function<void(const ignition::msgs::CameraInfo & msg)> camera_info_cb =
      [&](const ignition::msgs::CameraInfo & msg) {
        height = msg.height();
        width = msg.width();
        auto intrinsics = msg.intrinsics();
        for (auto i = 0; i < intrinsics.k_size(); ++i) {
          camera_k_[i] = intrinsics.k(i);
        }
        prom.set_value(true);
      };
    ign_node_->Subscribe(ign_camera_info_topic, camera_info_cb);
    if (future_result.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      std::cout << "failed to get camera info" << std::endl;
      return;
    }
    ign_node_->Unsubscribe(ign_camera_info_topic);
    node_->declare_parameter("camera_k", camera_k_);
    node_->declare_parameter("camera_d", camera_d_);
  }
  // create camera device
  cam_dev_ = std::make_shared<IgnCam>(ign_node_, ign_camera_image_topic, height, width);
  cam_server_ = std::make_shared<rmoss_cam::CamServer>(node_, cam_dev_);
}

}  // namespace rmoss_ign_cam

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmoss_ign_cam::IgnCamNode)
