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

#include "rmoss_ign_base/ign_joint_encoder.hpp"

#include <memory>
#include <string>
#include <cmath>


namespace rmoss_ign_base
{

IgnJointEncoder::IgnJointEncoder(
  const std::shared_ptr<ignition::transport::Node> & ign_node,
  const std::string & ign_joint_state_topic)
{
  ign_node_ = ign_node;
  ign_node_->Subscribe(ign_joint_state_topic, &IgnJointEncoder::ign_Joint_state_cb, this);
}

void IgnJointEncoder::ign_Joint_state_cb(const ignition::msgs::Model & msg)
{
  std::lock_guard<std::mutex> lock(msg_mut_);
  joint_state_msg_ = msg;
  // init index of joints in joint_state.
  if (!is_init_) {
    for (int i = 0; i < msg.joint_size(); i++) {
      joint_names_.push_back(msg.joint(i).name());
      joint_idx_map_[msg.joint(i).name()] = i;
    }
    is_init_ = true;
  }
}

int IgnJointEncoder::get_joint_idx(std::string & joint_name)
{
  if (is_init_) {
    if (joint_idx_map_.find(joint_name) != joint_idx_map_.end()) {
      return joint_idx_map_[joint_name];
    }
  }
  return -1;
}

double IgnJointEncoder::get_pos(int joint_idx)
{
  std::lock_guard<std::mutex> lock(msg_mut_);
  if (is_init_ && joint_idx >= 0 && joint_idx < joint_names_.size()) {
    return joint_state_msg_.joint(joint_idx).axis1().position();
  } else {
    return 0;
  }
}
double IgnJointEncoder::get_pos(std::string & joint_name)
{
  int idx = get_joint_idx(joint_name);
  return get_pos(idx);
}

double IgnJointEncoder::get_pitch()
{
  if (picth_idx_ < 0) {
    if (joint_idx_map_.find(pitch_name_) != joint_idx_map_.end()) {
      picth_idx_ = joint_idx_map_[pitch_name_];
    } else {
      return 0;
    }
  }
  return get_pos(picth_idx_);
}

double IgnJointEncoder::get_yaw()
{
  if (yaw_idx_ < 0) {
    if (joint_idx_map_.find(yaw_name_) != joint_idx_map_.end()) {
      yaw_idx_ = joint_idx_map_[yaw_name_];
    } else {
      return 0;
    }
  }
  return get_pos(yaw_idx_);
}


}  // namespace rmoss_ign_base
