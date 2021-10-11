/*******************************************************************************
 *  Copyright (c) 2020 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/

#ifndef IGNITION_GAZEBO_SYSTEMS_LIGHT_INDICATOR_HH
#define IGNITION_GAZEBO_SYSTEMS_LIGHT_INDICATOR_HH

#include <memory>
#include <ignition/gazebo/System.hh>

namespace ignition
{
    namespace gazebo
    {
        namespace systems
        {
            class LightIndicatorPrivate;
            class IGNITION_GAZEBO_VISIBLE LightIndicator
                : public ignition::gazebo::System,
                  public ISystemConfigure,
                  public ISystemPreUpdate
            {
            public:
                LightIndicator();
                ~LightIndicator() override = default;

            public:
                void Configure(const Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               EntityComponentManager &_ecm,
                               EventManager &_eventMgr) override;
                void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                               ignition::gazebo::EntityComponentManager &_ecm) override;
            private:
                std::unique_ptr<LightIndicatorPrivate> dataPtr;
            };
        } // namespace systems
    }     // namespace gazebo
} // namespace ignition

#endif //IGNITION_GAZEBO_SYSTEMS_LIGHT_INDICATOR_HH