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

#ifndef RMOSS_GZ_CAM__GZ_CAM_NODE_HPP_
#define RMOSS_GZ_CAM__GZ_CAM_NODE_HPP_

#include <string>
#include <memory>

#include "ignition/transport/Node.hh"
#include "rclcpp/rclcpp.hpp"
#include "rmoss_cam/cam_server.hpp"
#include "rmoss_gz_cam/gz_cam.hpp"

namespace rmoss_gz_cam
{
// Node wrapper for IgnCam.
class GzCamNode
{
public:
  explicit GzCamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ignition::transport::Node> gz_node_;
  std::shared_ptr<rmoss_cam::CamInterface> cam_dev_;
  std::shared_ptr<rmoss_cam::CamServer> cam_server_;
};

}  // namespace rmoss_gz_cam

#endif  // RMOSS_GZ_CAM__GZ_CAM_NODE_HPP_
