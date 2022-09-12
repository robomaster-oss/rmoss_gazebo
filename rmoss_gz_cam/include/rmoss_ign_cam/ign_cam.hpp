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

#ifndef RMOSS_IGN_CAM__IGN_CAM_HPP_
#define RMOSS_IGN_CAM__IGN_CAM_HPP_

#include <string>
#include <memory>
#include <unordered_map>

#include "opencv2/opencv.hpp"
#include "rmoss_cam/cam_interface.hpp"
#include "ignition/transport/Node.hh"

namespace rmoss_ign_cam
{
class IgnCam : public rmoss_cam::CamInterface
{
public:
  IgnCam(
    const std::shared_ptr<ignition::transport::Node> & ign_node,
    const std::string & topic_name,
    int height,
    int width);
  ~IgnCam();

public:
  bool open() override;
  bool close() override;
  bool is_open() override;
  bool grab_image(cv::Mat & image) override;
  bool set_parameter(rmoss_cam::CamParamType type, int value) override;
  bool get_parameter(rmoss_cam::CamParamType type, int & value) override;
  std::string error_message() override {return error_message_;}

private:
  void ign_image_cb(const ignition::msgs::Image & msg);

private:
  std::shared_ptr<ignition::transport::Node> ign_node_;
  std::string topic_name_;
  ignition::msgs::Image ign_msg_;
  bool grap_ok_{false};
  std::mutex msg_mut_;
  // camera parameters
  std::unordered_map<rmoss_cam::CamParamType, int> params_;
  std::string error_message_;
  // flag
  bool is_open_{false};
};

}  // namespace rmoss_ign_cam

#endif  // RMOSS_IGN_CAM__IGN_CAM_HPP_
