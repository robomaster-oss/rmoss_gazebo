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

#include "rmoss_ign_cam/ign_cam.hpp"

#include <string>
#include <memory>

#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

// copy from https://github.com/ignitionrobotics/ros_ign/blob/ros2/ros_ign_bridge/src/convert.cpp
void
convert_ign_to_ros(
  const ignition::msgs::Image & ign_msg,
  sensor_msgs::msg::Image & ros_msg)
{
  ros_msg.height = ign_msg.height();
  ros_msg.width = ign_msg.width();

  unsigned int num_channels;
  unsigned int octets_per_channel;

  if (ign_msg.pixel_format_type() == ignition::msgs::PixelFormatType::L_INT8) {
    ros_msg.encoding = "mono8";
    num_channels = 1;
    octets_per_channel = 1u;
  } else if (ign_msg.pixel_format_type() == ignition::msgs::PixelFormatType::L_INT16) {
    ros_msg.encoding = "mono16";
    num_channels = 1;
    octets_per_channel = 2u;
  } else if (ign_msg.pixel_format_type() == ignition::msgs::PixelFormatType::RGB_INT8) {
    ros_msg.encoding = "rgb8";
    num_channels = 3;
    octets_per_channel = 1u;
  } else if (ign_msg.pixel_format_type() == ignition::msgs::PixelFormatType::RGBA_INT8) {
    ros_msg.encoding = "rgba8";
    num_channels = 4;
    octets_per_channel = 1u;
  } else if (ign_msg.pixel_format_type() == ignition::msgs::PixelFormatType::BGRA_INT8) {
    ros_msg.encoding = "bgra8";
    num_channels = 4;
    octets_per_channel = 1u;
  } else if (ign_msg.pixel_format_type() == ignition::msgs::PixelFormatType::RGB_INT16) {
    ros_msg.encoding = "rgb16";
    num_channels = 3;
    octets_per_channel = 2u;
  } else if (ign_msg.pixel_format_type() == ignition::msgs::PixelFormatType::BGR_INT8) {
    ros_msg.encoding = "bgr8";
    num_channels = 3;
    octets_per_channel = 1u;
  } else if (ign_msg.pixel_format_type() == ignition::msgs::PixelFormatType::BGR_INT16) {
    ros_msg.encoding = "bgr16";
    num_channels = 3;
    octets_per_channel = 2u;
  } else if (ign_msg.pixel_format_type() == ignition::msgs::PixelFormatType::R_FLOAT32) {
    ros_msg.encoding = "32FC1";
    num_channels = 1;
    octets_per_channel = 4u;
  } else {
    std::cerr << "Unsupported pixel format [" << ign_msg.pixel_format_type() << "]" << std::endl;
    return;
  }

  ros_msg.is_bigendian = false;
  ros_msg.step = ros_msg.width * num_channels * octets_per_channel;

  auto count = ros_msg.step * ros_msg.height;
  ros_msg.data.resize(ros_msg.step * ros_msg.height);
  std::copy(
    ign_msg.data().begin(),
    ign_msg.data().begin() + count,
    ros_msg.data.begin());
}

namespace rmoss_ign_cam
{

IgnCam::IgnCam(
  const std::shared_ptr<ignition::transport::Node> & ign_node,
  const std::string & topic_name,
  int height,
  int width)
: ign_node_(ign_node), topic_name_(topic_name)
{
  params_[rmoss_cam::CamParamType::Fps] = 30;
  params_[rmoss_cam::CamParamType::Width] = width;
  params_[rmoss_cam::CamParamType::Height] = height;
}

IgnCam::~IgnCam()
{
  if (is_open_) {
    close();
  }
}

bool IgnCam::open()
{
  if (is_open_) {
    return true;
  }
  auto ret = ign_node_->Subscribe(topic_name_, &IgnCam::ign_image_cb, this);
  if (!ret) {
    error_message_ = "failed to create ignition subscriber";
    return false;
  }
  std::this_thread::sleep_for(1000ms);
  is_open_ = true;
  return true;
}

bool IgnCam::close()
{
  if (is_open_) {
    ign_node_->Unsubscribe(topic_name_);
    is_open_ = false;
  }
  return true;
}

bool IgnCam::is_open()
{
  return is_open_;
}

void IgnCam::ign_image_cb(const ignition::msgs::Image & msg)
{
  std::lock_guard<std::mutex> lock(msg_mut_);
  ign_msg_ = msg;
  grap_ok_ = true;
}

bool IgnCam::grab_image(cv::Mat & image)
{
  if (!is_open_) {
    error_message_ = "camera is not open";
    return false;
  }
  // wait image
  if (!grap_ok_) {
    int cnt = 0;
    while (!grap_ok_ && cnt < 20) {
      std::this_thread::sleep_for(5ms);
      cnt++;
    }
    if (!grap_ok_) {
      error_message_ = "grap timeout";
      return false;
    }
  }
  // copy image
  sensor_msgs::msg::Image ros_msg;
  {
    std::lock_guard<std::mutex> lock(msg_mut_);
    convert_ign_to_ros(ign_msg_, ros_msg);
  }
  image = cv_bridge::toCvCopy(ros_msg, "bgr8")->image.clone();
  grap_ok_ = false;
  return true;
}

// set and get parameter
bool IgnCam::set_parameter(rmoss_cam::CamParamType type, int value)
{
  if (type == rmoss_cam::CamParamType::Fps) {
    params_[type] = value;
    return true;
  }
  error_message_ = "only support Fps";
  return false;
}
bool IgnCam::get_parameter(rmoss_cam::CamParamType type, int & value)
{
  if (params_.find(type) != params_.end()) {
    value = params_[type];
    return true;
  } else {
    error_message_ = "";
    return false;
  }
}

}  // namespace rmoss_ign_cam
