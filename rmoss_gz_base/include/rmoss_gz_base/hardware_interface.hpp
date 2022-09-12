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

#ifndef RMOSS_GZ_BASE__HARDWARE_INTERFACE_HPP_
#define RMOSS_GZ_BASE__HARDWARE_INTERFACE_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace rmoss_gz_base
{

template<class DataT>
class Actuator
{
public:
  using SharedPtr = std::shared_ptr<Actuator<DataT>>;
  virtual void set(const DataT & data) = 0;
};

template<class DataT>
using SensorCallback = std::function<void (const DataT & data, const rclcpp::Time & stamp)>;

template<class DataT>
class Sensor
{
public:
  using SharedPtr = std::shared_ptr<Sensor<DataT>>;
  virtual void add_callback(SensorCallback<DataT> callback) = 0;
};

template<class DataT>
class DataSensor : public Sensor<DataT>
{
public:
  DataSensor() {}
  void add_callback(SensorCallback<DataT> callback) override
  {
    callbacks_.push_back(callback);
  }
  void update(const DataT & data, const rclcpp::Time & stamp)
  {
    if (callbacks_.size() > 0) {
      for (auto & cb : callbacks_) {
        cb(data, stamp);
      }
    }
  }
  std::vector<SensorCallback<DataT>> callbacks_;
};

}  // namespace rmoss_gz_base

#endif  // RMOSS_GZ_BASE__HARDWARE_INTERFACE_HPP_
