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

#ifndef GZ_SIM_SYSTEMS_LIGHT_BAR_CONTROLLER_HH
#define GZ_SIM_SYSTEMS_LIGHT_BAR_CONTROLLER_HH

#include <gz/sim/System.hh>
#include <memory>

namespace gz {
namespace sim {
namespace systems {
class LightBarControllerPrivate;
class GZ_SIM_VISIBLE LightBarController : public gz::sim::System,
                                          public ISystemConfigure,
                                          public ISystemPreUpdate {
public:
  LightBarController();
  ~LightBarController() override = default;

public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override;
  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

private:
  std::unique_ptr<LightBarControllerPrivate> dataPtr;
};
} // namespace systems
} // namespace sim
} // namespace gz

#endif // GZ_SIM_SYSTEMS_LIGHT_BAR_CONTROLLER_HH