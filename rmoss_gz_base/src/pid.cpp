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

#include "rmoss_gz_base/pid.hpp"

#include <string>

namespace rmoss_gz_base
{

void declare_pid_parameter(
  rclcpp::Node::SharedPtr node,
  const std::string & name)
{
  node->declare_parameter(name + ".p", 10.0);
  node->declare_parameter(name + ".i", 0.0);
  node->declare_parameter(name + ".d", 0.0);
  node->declare_parameter(name + ".imin", -1.0);
  node->declare_parameter(name + ".imax", 1.0);
  node->declare_parameter(name + ".cmdmin", -1000.0);
  node->declare_parameter(name + ".cmdmax", 1000.0);
  node->declare_parameter(name + ".offset", 0.0);
}

void declare_pid_parameter(
  rclcpp::Node::SharedPtr node,
  const std::string & name, PidParam & pid_param)
{
  node->declare_parameter(name + ".p", pid_param.p);
  node->declare_parameter(name + ".i", pid_param.i);
  node->declare_parameter(name + ".d", pid_param.d);
  node->declare_parameter(name + ".imin", pid_param.imin);
  node->declare_parameter(name + ".imax", pid_param.imin);
  node->declare_parameter(name + ".cmdmin", pid_param.cmdmin);
  node->declare_parameter(name + ".cmdmax", pid_param.cmdmax);
  node->declare_parameter(name + ".offset", pid_param.offset);
}

void get_pid_parameter(
  rclcpp::Node::SharedPtr node,
  const std::string & name, PidParam & pid_param)
{
  node->get_parameter(name + ".p", pid_param.p);
  node->get_parameter(name + ".i", pid_param.i);
  node->get_parameter(name + ".d", pid_param.d);
  node->get_parameter(name + ".imin", pid_param.imin);
  node->get_parameter(name + ".imax", pid_param.imin);
  node->get_parameter(name + ".cmdmin", pid_param.cmdmin);
  node->get_parameter(name + ".cmdmax", pid_param.cmdmax);
  node->get_parameter(name + ".offset", pid_param.offset);
}

}  // namespace rmoss_gz_base
