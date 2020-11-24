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
#include <mutex>
#include <queue>
#include <sstream>

#include <ignition/common/Profiler.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/Util.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

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
    ProjectileInfo(Entity _entity, std::string _name, std::chrono::steady_clock::duration _time)
        : entity(_entity)
        , name(_name)
        , spawnTime(_time)
        , isInit(false)
    {
    }
    ProjectileInfo()
        : isInit(false)
    {
    }
};

class ignition::gazebo::systems::ProjectileShooterPrivate {
public:
    void OnCmd(const ignition::msgs::Int32& _msg);
    void PreUpdate(const ignition::gazebo::UpdateInfo& _info, ignition::gazebo::EntityComponentManager& _ecm);
    void PostUpdate(const ignition::gazebo::UpdateInfo& _info, const ignition::gazebo::EntityComponentManager& _ecm);

public:
    transport::Node node;
    std::unique_ptr<SdfEntityCreator> creator { nullptr };
    Entity worldEntity { kNullEntity };
    //current model and shooter link
    Model model { kNullEntity };
    Entity shooterEntity { kNullEntity };
    //general data
    std::string modelName;
    std::string shooterName { "shooter" };
    sdf::Root projectileSdfRoot;
    unsigned int projectileId = 0;
    //control data
    double projectileVel { 20 };
    int currentTotalNum { 1000000 };
    int waitShootNum { 0 };
    std::mutex waitShootNumMutex;
    std::queue<ProjectileInfo> spawnedProjectiles;
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
    if (_sdf->HasElement("shooter_name")) {
        this->dataPtr->shooterName = _sdf->Get<std::string>("shooter_name");
    }
    if (_sdf->HasElement("projectile_velocity")) {
        this->dataPtr->projectileVel = _sdf->Get<double>("projectile_velocity");
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

int i = 0;
void ProjectileShooterPrivate::PreUpdate(const ignition::gazebo::UpdateInfo& _info, ignition::gazebo::EntityComponentManager& _ecm)
{
    //do nothing if paused.
    if (_info.paused) {
        return;
    }
    //for test
    i++;
    if (i == 4000) {
        this->waitShootNum = 1;
    } else if (i % 4000 == 0) {
        this->waitShootNum = 1;
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
    if (this->currentTotalNum < 0) {
        spawnFlag = false;
    }
    // wait the last projectile init completely
    if (!this->spawnedProjectiles.empty()) {
        ProjectileInfo& last = this->spawnedProjectiles.back();
        double t = std::chrono::duration_cast<std::chrono::milliseconds>(_info.simTime - last.spawnTime).count();
        if (last.isInit && t > 100) {
            spawnFlag = true;
        } else {
            spawnFlag = false;
        }
    } else {
        spawnFlag = true;
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
        math::Pose3d shooterOffset(0.2, 0, 0, 0, 0, 0);
        sdf::Model modelToSpawn = *this->projectileSdfRoot.ModelByIndex(0);
        modelToSpawn.SetName(projectileName);
        modelToSpawn.SetRawPose(shooterPose * shooterOffset);
        Entity entity = this->creator->CreateEntities(&modelToSpawn);
        this->creator->SetParent(entity, this->worldEntity);
        //set velocity
        math::Vector3d tmpVel(this->projectileVel, 0, 0);
        _ecm.CreateComponent(entity, components::LinearVelocityCmd({ tmpVel }));
        ProjectileInfo pInfo(entity, projectileName, _info.simTime);
        this->spawnedProjectiles.push(pInfo);
        this->projectileId++;
        {
            std::lock_guard<std::mutex> lock(this->waitShootNumMutex);
            this->waitShootNum--;
        }
        this->currentTotalNum--;
    } else {
        //try to init velocity
        if (!this->spawnedProjectiles.empty()) {
            ProjectileInfo& pInfo = this->spawnedProjectiles.back();
            if (!pInfo.isInit) {
                _ecm.RemoveComponent<components::LinearVelocityCmd>(pInfo.entity);
                pInfo.isInit = true;
            }
        }
    }
    //check time and delete projectiles
    if (!this->spawnedProjectiles.empty()) {
        ProjectileInfo& pInfo = this->spawnedProjectiles.front();
        double t = std::chrono::duration_cast<std::chrono::milliseconds>(_info.simTime - pInfo.spawnTime).count();
        if (pInfo.isInit && t > 5000) {
            this->creator->RequestRemoveEntity(pInfo.entity);
            this->spawnedProjectiles.pop();
        }
    }
}

void ProjectileShooterPrivate::PostUpdate(const ignition::gazebo::UpdateInfo& _info,
    const ignition::gazebo::EntityComponentManager& _ecm)
{
}

/******************register*************************************************/
IGNITION_ADD_PLUGIN(ProjectileShooter,
    ignition::gazebo::System,
    ProjectileShooter::ISystemConfigure,
    ProjectileShooter::ISystemPreUpdate,
    ProjectileShooter::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ProjectileShooter, "ignition::gazebo::systems::ProjectileShooter")