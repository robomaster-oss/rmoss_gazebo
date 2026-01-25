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

#include <list>
#include <mutex>
#include <sstream>

#include <gz/common/Profiler.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>

#include <gz/sim/Conversions.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/Util.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include "ProjectileShooter.hh"
#include <sdf/Model.hh>
#include <sdf/Root.hh>

using namespace gz;
using namespace sim;
using namespace systems;

struct ProjectileInfo {
  Entity entity;
  std::string name;
  std::chrono::steady_clock::duration spawnTime;
  bool isInit;
  bool remove{false};
  ProjectileInfo(Entity _entity, std::string _name,
                 std::chrono::steady_clock::duration _time)
      : entity(_entity), name(_name), spawnTime(_time), isInit(false) {}
  ProjectileInfo() : isInit(false) {}
};

class gz::sim::systems::ProjectileShooterPrivate {
public:
  void OnCmd(const gz::msgs::Int32 &_msg);
  void OnSetVel(const gz::msgs::Double &_msg);
  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm);
  void publishAttackInfo(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm,
                         Entity _targetCollision);
  void publishShootInfo(const gz::sim::UpdateInfo &_info);

public:
  // node and tool
  transport::Node node;
  transport::Node::Publisher attackInfoPub;
  transport::Node::Publisher shootInfoPub;
  std::unique_ptr<SdfEntityCreator> creator{nullptr};
  bool initialized{false};
  // entity: world,model, shooter link
  Entity world{kNullEntity};
  Entity model{kNullEntity};
  Entity shooterLink{kNullEntity};
  std::string modelName;
  std::string shooterName{"default_shooter"};
  math::Pose3d shooterOffset;
  // parameters of shooter
  double shootVel{20};
  double shootPeriodMS{50};
  // projectile
  sdf::Model projectileSdfModel;
  unsigned int projectileId{0};
  int waitShootNum{0};
  std::mutex waitShootNumMutex;
  std::list<ProjectileInfo> spawnedProjectiles;
  std::chrono::steady_clock::duration lastSpawnTime;
};

/******************implementation for ProjectileShooter************************/
ProjectileShooter::ProjectileShooter()
    : dataPtr(std::make_unique<ProjectileShooterPrivate>()) {}

void ProjectileShooter::Configure(
    const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &_eventMgr) {
  this->dataPtr->model = _entity;
  auto modelWrapper = Model(_entity);
  if (!modelWrapper.Valid(_ecm)) {
    gzerr << "ProjectileShooter plugin should be attached to a model entity. \
                     Failed to initialize."
          << std::endl;
    return;
  }
  this->dataPtr->modelName = modelWrapper.Name(_ecm);
  // Get params from SDF
  auto linkName = _sdf->Get<std::string>("shooter_link");
  this->dataPtr->shooterLink = modelWrapper.LinkByName(_ecm, linkName);
  if (this->dataPtr->shooterLink == kNullEntity) {
    gzerr << "shooter link with name[" << linkName << "] not found. "
          << std::endl;
    return;
  }
  if (_sdf->HasElement("shooter_name")) {
    this->dataPtr->shooterName = _sdf->Get<std::string>("shooter_name");
  }
  if (_sdf->HasElement("shooter_offset")) {
    this->dataPtr->shooterOffset = _sdf->Get<math::Pose3d>("shooter_offset");
  } else {
    gzerr << "The tag <shooter_offset> is not found. " << std::endl;
    return;
  }
  if (_sdf->HasElement("projectile_velocity")) {
    this->dataPtr->shootVel = _sdf->Get<double>("projectile_velocity");
  }
  std::string projectile_uri;
  if (_sdf->HasElement("projectile_uri")) {
    projectile_uri = _sdf->Get<std::string>("projectile_uri");
  } else {
    gzerr << "The tag <projectile_uri> is not found." << std::endl;
    return;
  }
  std::string shootInfoTopic = "/referee_system/shoot_info";
  if (_sdf->HasElement("shoot_info_topic")) {
    shootInfoTopic = _sdf->Get<std::string>("shoot_info_topic");
  }
  std::string attackInfoTopic = "/referee_system/attack_info";
  if (_sdf->HasElement("attack_info_topic")) {
    attackInfoTopic = _sdf->Get<std::string>("attack_info_topic");
  }
  // Load projectile model sdf
  gz::common::SystemPaths systemPaths;
  sdf::Root projectileSdfRoot;
  sdf::Errors errors =
      projectileSdfRoot.Load(systemPaths.FindFileURI(projectile_uri));
  if (!errors.empty()) {
    gzerr << "Load Projectile Model:" << projectile_uri << std::endl;
    for (const auto &e : errors) {
      gzerr << e.Message() << std::endl;
    }
    return;
  }
  if (projectileSdfRoot.Model() == nullptr) {
    gzerr << "Projectile Model not found" << std::endl;
    return;
  }
  this->dataPtr->projectileSdfModel = *projectileSdfRoot.Model();
  // Subscribe to commands
  std::string shootCmdTopic{this->dataPtr->modelName + "/" +
                            this->dataPtr->shooterName + "/shoot"};
  std::string setVelTopic{this->dataPtr->modelName + "/" +
                          this->dataPtr->shooterName + "/set_vel"};
  this->dataPtr->node.Subscribe(shootCmdTopic, &ProjectileShooterPrivate::OnCmd,
                                this->dataPtr.get());
  this->dataPtr->node.Subscribe(
      setVelTopic, &ProjectileShooterPrivate::OnSetVel, this->dataPtr.get());
  // Publisher
  this->dataPtr->shootInfoPub =
      this->dataPtr->node.Advertise<msgs::StringMsg>(shootInfoTopic);
  this->dataPtr->attackInfoPub =
      this->dataPtr->node.Advertise<msgs::StringMsg>(attackInfoTopic);
  // creator and world
  this->dataPtr->creator = std::make_unique<SdfEntityCreator>(_ecm, _eventMgr);
  this->dataPtr->world = _ecm.EntityByComponents(components::World());
  this->dataPtr->initialized = true;
  // debug info
  gzdbg << "[" << this->dataPtr->modelName
        << " ProjectileShooter Info]:" << std::endl;
  gzdbg << "Shooter name: " << this->dataPtr->shooterName << std::endl;
  gzdbg << "Shooter offset: " << this->dataPtr->shooterOffset << std::endl;
  gzdbg << "Projectile name: " << this->dataPtr->projectileSdfModel.Name()
        << std::endl;
  gzdbg << "Shoot CMD Topic: " << shootCmdTopic << std::endl;
  gzdbg << "Set Vel Topic: " << setVelTopic << std::endl;
  gzdbg << "Shoot Info Topic: " << shootInfoTopic << std::endl;
  gzdbg << "Attack Info Topic: " << attackInfoTopic << std::endl;
}

void ProjectileShooter::PreUpdate(const gz::sim::UpdateInfo &_info,
                                  gz::sim::EntityComponentManager &_ecm) {
  this->dataPtr->PreUpdate(_info, _ecm);
}

/******************implementation for
 * ProjectileShooterPrivate******************/

void ProjectileShooterPrivate::OnCmd(const gz::msgs::Int32 &_msg) {
  std::lock_guard<std::mutex> lock(this->waitShootNumMutex);
  if (_msg.data() >= 0) {
    this->waitShootNum = _msg.data();
  }
  // gzdbg << "ProjectileShooter OnCmd msg: [" << _msg.data() << "]" <<
  // std::endl;
}

void ProjectileShooterPrivate::OnSetVel(const gz::msgs::Double &_msg) {
  std::lock_guard<std::mutex> lock(this->waitShootNumMutex);
  if (_msg.data() > 0 && _msg.data() < 30) {
    this->shootVel = _msg.data();
  }
  // gzdbg << "ProjectileShooter OnSetVel msg: [" << _msg.data() << "]" <<
  // std::endl;
}

void ProjectileShooterPrivate::PreUpdate(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {
  // do nothing if it's paused or not initialized.
  if (_info.paused || !this->initialized) {
    return;
  }
  // Pose of shooter Link
  if (!_ecm.Component<components::WorldPose>(this->shooterLink)) {
    _ecm.CreateComponent(this->shooterLink, components::WorldPose());
  }
  // lock
  std::lock_guard<std::mutex> lock(this->waitShootNumMutex);
  // spawnFlag CMD
  bool spawnFlag = false;
  double t = std::chrono::duration_cast<std::chrono::milliseconds>(
                 _info.simTime - this->lastSpawnTime)
                 .count();
  if (this->waitShootNum > 0 && t > this->shootPeriodMS) {
    if (this->spawnedProjectiles.empty() ||
        this->spawnedProjectiles.back().isInit) {
      this->lastSpawnTime = _info.simTime;
      spawnFlag = true;
    }
  }
  // process projectile
  if (spawnFlag) {
    // spawn a projectile
    auto projectileName = this->modelName + "_" + this->shooterName + "_" +
                          std::to_string(this->projectileId);
    this->projectileSdfModel.SetName(projectileName);
    // current state
    auto shooterPose =
        _ecm.Component<components::WorldPose>(this->shooterLink)->Data();
    this->projectileSdfModel.SetRawPose(shooterPose * this->shooterOffset);
    Entity projectileModel =
        this->creator->CreateEntities(&this->projectileSdfModel);
    this->creator->SetParent(projectileModel, this->world);
    // update projectile,set velocity and create ContactSensorData
    math::Vector3d tmpVel(this->shootVel, 0, 0);
    _ecm.CreateComponent(projectileModel,
                         components::LinearVelocityCmd({tmpVel}));
    Entity projectileLink = Model(projectileModel).Links(_ecm)[0];
    Entity projectileCollision = Link(projectileLink).Collisions(_ecm)[0];
    _ecm.CreateComponent(projectileCollision, components::ContactSensorData());
    // update queue and counters
    ProjectileInfo pInfo(projectileModel, projectileName, _info.simTime);
    this->spawnedProjectiles.push_back(pInfo);
    this->projectileId++;
    this->waitShootNum--;
    // publish info
    this->publishShootInfo(_info);
  } else {
    // try to init the last projectile
    if (!this->spawnedProjectiles.empty()) {
      ProjectileInfo &pInfo = this->spawnedProjectiles.back();
      if (!pInfo.isInit) {
        // cancel velocity
        _ecm.RemoveComponent<components::LinearVelocityCmd>(pInfo.entity);
        pInfo.isInit = true;
      }
    }
  }
  // check
  if (!this->spawnedProjectiles.empty()) {
    // check cantact and time to delete projectiles
    for (auto &pInfo : this->spawnedProjectiles) {
      if (!pInfo.isInit) {
        continue;
      }
      Entity link = Model(pInfo.entity).Links(_ecm)[0];
      Entity collision = Link(link).Collisions(_ecm)[0];
      auto contacts =
          _ecm.Component<components::ContactSensorData>(collision)->Data();
      if (contacts.contact_size() > 0) {
        Entity collision1 = contacts.contact(0).collision1().id();
        Entity collision2 = contacts.contact(0).collision2().id();
        Entity targetCollision =
            (collision == collision1) ? collision2 : collision1;
        this->publishAttackInfo(_info, _ecm, targetCollision);
        // gzmsg << "ProjectileShooter contact_size: [" <<
        // contacts.contact_size() << "]" << std::endl;
      }
      double t = std::chrono::duration_cast<std::chrono::milliseconds>(
                     _info.simTime - pInfo.spawnTime)
                     .count();
      if (contacts.contact_size() > 0 || t > 4000) {
        this->creator->RequestRemoveEntity(pInfo.entity);
        pInfo.remove = true;
      }
    }
    // remove from list
    spawnedProjectiles.remove_if(
        [](const ProjectileInfo &pInfo) { return pInfo.remove; });
  }
}

void ProjectileShooterPrivate::publishAttackInfo(
    const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm,
    Entity _targetCollision) {
  std::string target = scopedName(_targetCollision, _ecm, "/", false);
  std::string result = this->modelName + "/" + this->shooterName + "," + target;
  // publish msg
  msgs::StringMsg msg;
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
  msg.set_data(result);
  this->attackInfoPub.Publish(msg);
  // gzmsg << "ProjectileShooter AttackInfo : [" << result << "]" << std::endl;
}

void ProjectileShooterPrivate::publishShootInfo(
    const gz::sim::UpdateInfo &_info) {
  // publish msg
  std::string result = this->modelName + "/" + this->shooterName + "," +
                       std::to_string(this->shootVel);
  msgs::StringMsg msg;
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
  msg.set_data(result);
  this->shootInfoPub.Publish(msg);
  // gzmsg << "ProjectileShooter ShootInfo : [" << result << "]" << std::endl;
}

/******************register*************************************************/
GZ_ADD_PLUGIN(ProjectileShooter, gz::sim::System,
              ProjectileShooter::ISystemConfigure,
              ProjectileShooter::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ProjectileShooter, "gz::sim::systems::ProjectileShooter")