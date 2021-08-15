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


#ifndef IGNITION_GAZEBO_SYSTEMS_PROJECTILE_SHOOTER_HH
#define IGNITION_GAZEBO_SYSTEMS_PROJECTILE_SHOOTER_HH

#include <memory>
#include <ignition/gazebo/System.hh>

namespace ignition
{
    namespace gazebo
    {
        namespace systems
        {
            class ProjectileShooterPrivate;
            class IGNITION_GAZEBO_VISIBLE ProjectileShooter
                : public ignition::gazebo::System,
                  public ISystemConfigure,
                  public ISystemPreUpdate
            {
            public:
                ProjectileShooter();
                ~ProjectileShooter() override = default;

            public:
                void Configure(const Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               EntityComponentManager &_ecm,
                               EventManager &_eventMgr) override;
                void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                               ignition::gazebo::EntityComponentManager &_ecm) override;

            private:
                std::unique_ptr<ProjectileShooterPrivate> dataPtr;
            };
        } // namespace systems
    }     // namespace gazebo
} // namespace ignition

#endif //IGNITION_GAZEBO_SYSTEMS_PROJECTILE_SHOOTER_HH