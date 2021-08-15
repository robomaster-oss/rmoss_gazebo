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

#ifndef RMOSS_IGN_BASE__IGN_JOINT_ENCODER_HPP_
#define RMOSS_IGN_BASE__IGN_JOINT_ENCODER_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <map>
#include <vector>

#include "ignition/transport/Node.hh"

namespace rmoss_ign_base
{

class IgnJointEncoder
{
public:
  IgnJointEncoder(
    const std::shared_ptr<ignition::transport::Node> & ign_node,
    const std::string & ign_joint_state_topic);
  ~IgnJointEncoder() {}

public:
  bool is_init() {return is_init_;}
  // get position of joint
  int get_joint_idx(std::string & joint_name);
  double get_pos(int joint_idx);
  double get_pos(std::string & joint_name);
  // extra API for pitch and yaw
  void set_pitch_joint_name(const std::string & joint_name) {yaw_name_ = joint_name;}
  void set_yaw_joint_name(const std::string & joint_name) {pitch_name_ = joint_name;}
  double get_pitch();
  double get_yaw();

private:
  void ign_Joint_state_cb(const ignition::msgs::Model & msg);

private:
  std::shared_ptr<ignition::transport::Node> ign_node_;
  // sensor data
  ignition::msgs::Model joint_state_msg_;
  std::mutex msg_mut_;
  // info
  std::vector<std::string> joint_names_;
  std::map<std::string, int> joint_idx_map_;
  bool is_init_{false};
  // extra
  std::string yaw_name_{"gimbal_yaw_joint"};
  std::string pitch_name_{"gimbal_pitch_joint"};
  int yaw_idx_{-1};
  int picth_idx_{-1};
};

}  // namespace rmoss_ign_base

#endif  // RMOSS_IGN_BASE__IGN_JOINT_ENCODER_HPP_
