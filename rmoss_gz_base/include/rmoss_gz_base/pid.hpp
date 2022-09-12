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

#ifndef RMOSS_GZ_BASE__PID_HPP_
#define RMOSS_GZ_BASE__PID_HPP_

#include <string>

#include "ignition/math/PID.hh"
#include "rclcpp/rclcpp.hpp"

namespace rmoss_gz_base
{

struct PidParam
{
  double p;
  double i;
  double d;
  double imax;
  double imin;
  double cmdmin;
  double cmdmax;
  double offset;
};

void declare_pid_parameter(
  rclcpp::Node::SharedPtr node,
  const std::string & name);

void declare_pid_parameter(
  rclcpp::Node::SharedPtr node,
  const std::string & name, PidParam & pid_param);

void get_pid_parameter(
  rclcpp::Node::SharedPtr node,
  const std::string & name, PidParam & pid_param);

}  // namespace rmoss_gz_base

#endif  // RMOSS_GZ_BASE__PID_HPP_
