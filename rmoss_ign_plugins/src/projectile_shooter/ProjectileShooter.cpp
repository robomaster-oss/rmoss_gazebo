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
#include <ignition/common/Profiler.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/Util.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <mutex>
#include <sstream>

#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include "ProjectileShooter.hh"
#include <sdf/Model.hh>
#include <sdf/Root.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;

struct ProjectileInfo {
    Entity entity;
    std::string name;
    std::chrono::steady_clock::duration spawnTime;
    bool isInit;
    bool isDeleted;
    ProjectileInfo(Entity _entity, std::string _name, std::chrono::steady_clock::duration _time)
        : entity(_entity)
        , name(_name)
        , spawnTime(_time)
        , isInit(false)
        , isDeleted(false)
    {
    }
    ProjectileInfo()
        : isInit(false)
        , isDeleted(false)
    {
    }
};

class ignition::gazebo::systems::ProjectileShooterPrivate {
public:
    void OnCmd(const ignition::msgs::Int32& _msg);
    void PreUpdate(const ignition::gazebo::UpdateInfo& _info, ignition::gazebo::EntityComponentManager& _ecm);
    void PostUpdate(const ignition::gazebo::UpdateInfo& _info, const ignition::gazebo::EntityComponentManager& _ecm);

public:
    std::string shooterName;
    transport::Node node;
    std::unique_ptr<SdfEntityCreator> creator { nullptr };
    Entity worldEntity { kNullEntity };
    //model
    std::string modelName;
    Model model { kNullEntity };
    //shooter link
    Entity shooterEntity { kNullEntity };
    //data
    sdf::Root projectileSdfRoot;
    unsigned int projectileId = 0;
    int currentTotalNum { 0 };
    int waitShootNum { 0 };
    std::vector<ProjectileInfo> spawnedProjectiles;
    std::mutex waitShootNumMutex;
};

/******************implementation for ProjectileShooter************************/
ProjectileShooter::ProjectileShooter()
    : dataPtr(std::make_unique<ProjectileShooterPrivate>())
{
}

void ProjectileShooter::Configure(const Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    EntityComponentManager& _ecm,
    EventManager& _eventMgr)
{
    this->dataPtr->model = Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm)) {
        ignerr << "ProjectileShooter plugin should be attached to a model entity. Failed to initialize." << std::endl;
        return;
    }
    // Get params from SDF
    this->dataPtr->shooterName = "shooter";
    if (_sdf->HasElement("shooter_name")) {
        this->dataPtr->shooterName = _sdf->Get<std::string>("shooter_name");
    }
    // Get shooter link
    auto linkName = _sdf->Get<std::string>("shooter_link");
    this->dataPtr->shooterEntity = this->dataPtr->model.LinkByName(_ecm, linkName);
    if (this->dataPtr->shooterEntity == kNullEntity) {
        ignerr << "shooter link with name[" << linkName << "] not found. " << std::endl;
        return;
    }
    // Load projectile model sdf
    std::string uri = "model://rm_projectile17/model.sdf";
    ignition::common::SystemPaths systemPaths;
    auto model_path = systemPaths.FindFileURI(uri);
    sdf::Errors errors = this->dataPtr->projectileSdfRoot.Load(model_path);
    if (!errors.empty()) {
        for (const auto& e : errors) {
            ignerr << e.Message() << std::endl;
        }
        return;
    }
    if (this->dataPtr->projectileSdfRoot.ModelCount() == 0) {
        ignerr << "Projectile Model not found" << std::endl;
        return;
    }
    ignmsg << "load projectile name:[" << this->dataPtr->projectileSdfRoot.ModelByIndex(0)->Name() << "]" << std::endl;
    this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);
    // Subscribe to commands
    std::string topic { this->dataPtr->modelName + "/" + this->dataPtr->shooterName + "/shoot" };
    this->dataPtr->node.Subscribe(topic, &ProjectileShooterPrivate::OnCmd, this->dataPtr.get());
    ignmsg << "[" << this->dataPtr->shooterName << "] subscribing to shoot messages on [" << topic << "]" << std::endl;
    //publish status of shooter
    //creator of projectile
    this->dataPtr->creator = std::make_unique<SdfEntityCreator>(_ecm, _eventMgr);
    this->dataPtr->worldEntity = _ecm.EntityByComponents(components::World());
    this->dataPtr->waitShootNum = 100;
}

void ProjectileShooter::PreUpdate(const ignition::gazebo::UpdateInfo& _info,
    ignition::gazebo::EntityComponentManager& _ecm)
{
    this->dataPtr->PreUpdate(_info, _ecm);
}
void ProjectileShooter::PostUpdate(const ignition::gazebo::UpdateInfo& _info,
    const ignition::gazebo::EntityComponentManager& _ecm)
{
    this->dataPtr->PostUpdate(_info, _ecm);
}

/******************implementation for ProjectileShooterPrivate******************/

void ProjectileShooterPrivate::OnCmd(const ignition::msgs::Int32& _msg)
{
    std::lock_guard<std::mutex> lock(this->waitShootNumMutex);
    this->waitShootNum = _msg.data();
    ignmsg << "ProjectileShooter msg x: [" << _msg.data() << "]" << std::endl;
}

void ProjectileShooterPrivate::PreUpdate(const ignition::gazebo::UpdateInfo& _info, ignition::gazebo::EntityComponentManager& _ecm)
{
    //do nothing if paused.
    if (_info.paused) {
        return;
    }
    //control for chassis
    Link shooterEntity(this->shooterEntity);
    if (!_ecm.Component<components::WorldPose>(this->shooterEntity)) {
        _ecm.CreateComponent(this->shooterEntity, components::WorldPose());
    }
    if (!_ecm.Component<components::LinearVelocity>(this->shooterEntity)) {
        _ecm.CreateComponent(this->shooterEntity, components::LinearVelocity());
    }
    if (!_ecm.Component<components::AngularVelocity>(this->shooterEntity)) {
        _ecm.CreateComponent(this->shooterEntity, components::AngularVelocity());
    }
    // current state
    const auto shooterPose = _ecm.Component<components::WorldPose>(this->shooterEntity)->Data();
    const auto linearVel = _ecm.Component<components::LinearVelocity>(this->shooterEntity)->Data();
    const auto angularVel = _ecm.Component<components::AngularVelocity>(this->shooterEntity)->Data();
    // check spawn CMD
    bool spawnFlag = true;
    // wait the last projectile init completely
    if (this->spawnedProjectiles.size() > 0 && (!this->spawnedProjectiles.back().isInit)) {
        spawnFlag = false;
    }
    if (spawnFlag) {
        std::lock_guard<std::mutex> lock(this->waitShootNumMutex);
        if (this->waitShootNum <= 0) {
            spawnFlag = false;
        }
    }
    //process projectile
    if (spawnFlag) {
        //spawn a projectiles
        auto projectileName = this->modelName + "_" + this->shooterName + "_" + std::to_string(this->projectileId);
        math::Pose3d shooterOffset(0.3, 0, 0, 0, 0, 0);
        sdf::Model modelToSpawn = *this->projectileSdfRoot.ModelByIndex(0);
        modelToSpawn.SetName(projectileName);
        modelToSpawn.SetRawPose(shooterPose + shooterOffset);
        Entity entity = this->creator->CreateEntities(&modelToSpawn);
        this->creator->SetParent(entity, this->worldEntity);
        // if (!_ecm.Component<components::LinearVelocityCmd>(entity)) {
        //     _ecm.CreateComponent(entity, components::LinearVelocityCmd());
        // }
        math::Vector3d velocity(1, 0, 0);
        _ecm.CreateComponent(entity, components::LinearVelocityCmd({ velocity }));
        ProjectileInfo pInfo(entity, projectileName, _info.simTime);
        this->spawnedProjectiles.push_back(pInfo);
        this->projectileId++;
        {
            std::lock_guard<std::mutex> lock(this->waitShootNumMutex);
            this->waitShootNum--;
        }
    } else {
        //try to init velocity
        if (this->spawnedProjectiles.size() > 0) {
            ProjectileInfo& pInfo = this->spawnedProjectiles.back();
            if (!pInfo.isInit) {
                _ecm.RemoveComponent<components::LinearVelocityCmd>(pInfo.entity);
                pInfo.isInit = true;
            }
        }
    }
    //check time and delete projectiles
}

void ProjectileShooterPrivate::PostUpdate(const ignition::gazebo::UpdateInfo& _info,
    const ignition::gazebo::EntityComponentManager& _ecm)
{
    // Entity entity=this->test;
    // if(entity!=kNullEntity){
    //     _ecm.RemoveComponent<components::LinearVelocityCmd>(entity);
    // }
}

/******************register*************************************************/
IGNITION_ADD_PLUGIN(ProjectileShooter,
    ignition::gazebo::System,
    ProjectileShooter::ISystemConfigure,
    ProjectileShooter::ISystemPreUpdate,
    ProjectileShooter::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ProjectileShooter, "ignition::gazebo::systems::ProjectileShooter")