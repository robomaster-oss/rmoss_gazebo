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

#include "ignition/gazebo/components/ContactSensorData.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>

#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include <ignition/gazebo/Util.hh>

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
    // node and tool
    transport::Node node;
    transport::Node::Publisher attackPub;
    std::unique_ptr<SdfEntityCreator> creator { nullptr };
    bool initialized { false };
    // entity: world,model, shooter link
    Entity world { kNullEntity };
    Entity model { kNullEntity };
    Entity shooterLink { kNullEntity };
    std::string modelName;
    std::string shooterName;
    math::Pose3d shooterOffset;
    // projectile
    sdf::Model projectileSdfModel;
    unsigned int projectileId = 0;
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
    this->dataPtr->model = _entity;
    auto modelWrapper = Model(_entity);
    if (!modelWrapper.Valid(_ecm)) {
        ignerr << "ProjectileShooter plugin should be attached to a model entity. Failed to initialize." << std::endl;
        return;
    }
    this->dataPtr->modelName = modelWrapper.Name(_ecm);
    // Get params from SDF
    auto linkName= _sdf->Get<std::string>("shooter_link");
    this->dataPtr->shooterLink = modelWrapper.LinkByName(_ecm, linkName);
    if (this->dataPtr->shooterLink == kNullEntity) {
        ignerr << "shooter link with name[" << linkName << "] not found. " << std::endl;
        return;
    }
    if (_sdf->HasElement("shooter_name")) {
        this->dataPtr->shooterName = _sdf->Get<std::string>("shooter_name");
    }
    this->dataPtr->shooterOffset = math::Pose3d(0.2, 0, 0, 0, 0, 0);
    if (_sdf->HasElement("shooter_offset")) {
        this->dataPtr->shooterOffset = _sdf->Get<math::Pose3d>("shooter_offset");
    }
    if (_sdf->HasElement("projectile_velocity")) {
        this->dataPtr->projectileVel = _sdf->Get<double>("projectile_velocity");
    }
    if (_sdf->HasElement("projectile_num")) {
        this->dataPtr->currentTotalNum = _sdf->Get<double>("projectile_num");
    }
    std::string projectile_uri;
    if (_sdf->HasElement("projectile_uri")) {
        projectile_uri = _sdf->Get<std::string>("projectile_uri");
    } else {
        ignerr << "The tag <projectile_uri> is not found." << std::endl;
        return;
    }
    std::string attackTopic = "/referee_system/attack_info";
    if (_sdf->HasElement("attack_topic")) {
        attackTopic = _sdf->Get<std::string>("attack_topic");
    }
    // Load projectile model sdf
    ignition::common::SystemPaths systemPaths;
    sdf::Root projectileSdfRoot;
    sdf::Errors errors = projectileSdfRoot.Load(systemPaths.FindFileURI(projectile_uri));
    if (!errors.empty()) {
        for (const auto& e : errors) {
            ignerr << e.Message() << std::endl;
        }
        return;
    }
    if (projectileSdfRoot.ModelCount() == 0) {
        ignerr << "Projectile Model not found" << std::endl;
        return;
    }
    this->dataPtr->projectileSdfModel = *projectileSdfRoot.ModelByIndex(0);
    // Subscribe to commands
    std::string shootTopic { this->dataPtr->modelName + "/" + this->dataPtr->shooterName + "/shoot" };
    this->dataPtr->node.Subscribe(shootTopic, &ProjectileShooterPrivate::OnCmd, this->dataPtr.get());
    this->dataPtr->attackPub = this->dataPtr->node.Advertise<msgs::StringMsg>(attackTopic);
    //creator and world
    this->dataPtr->creator = std::make_unique<SdfEntityCreator>(_ecm, _eventMgr);
    this->dataPtr->world = _ecm.EntityByComponents(components::World());
    this->dataPtr->initialized = true;
    //debug info
    igndbg << "[" << this->dataPtr->modelName << " ProjectileShooter info]:" << std::endl;
    igndbg << "shooter name: " << this->dataPtr->shooterName << std::endl;
    igndbg << "shooter offset: " << this->dataPtr->shooterOffset << std::endl;
    igndbg << "projectile name: " << projectileSdfRoot.ModelByIndex(0)->Name() << std::endl;
    igndbg << "shoot topic: " << shootTopic << std::endl;
    igndbg << "attack topic: " << attackTopic << std::endl;
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
    //ignmsg << "ProjectileShooter msg x: [" << _msg.data() << "]" << std::endl;
}

void ProjectileShooterPrivate::PreUpdate(const ignition::gazebo::UpdateInfo& _info, ignition::gazebo::EntityComponentManager& _ecm)
{
    //do nothing if it's paused or not initialized.
    if (_info.paused || !this->initialized) {
        return;
    }
    //Pose of shooter Link
    if (!_ecm.Component<components::WorldPose>(this->shooterLink)) {
        _ecm.CreateComponent(this->shooterLink, components::WorldPose());
    }
    // current state
    const auto shooterPose = _ecm.Component<components::WorldPose>(this->shooterLink)->Data();
    // spawnFlag CMD
    bool spawnFlag = false;
    {
        std::lock_guard<std::mutex> lock(this->waitShootNumMutex);
        if (this->currentTotalNum > 0 && this->waitShootNum > 0) {
            if (!this->spawnedProjectiles.empty()) {
                //the last projectile finished to init.
                ProjectileInfo& last = this->spawnedProjectiles.back();
                double t = std::chrono::duration_cast<std::chrono::milliseconds>(_info.simTime - last.spawnTime).count();
                if (last.isInit && t > 100) {
                    spawnFlag = true;
                }
            } else {
                //the first projectile
                spawnFlag = true;
            }
        }
    }
    //process projectile
    if (spawnFlag) {
        //spawn a projectile
        auto projectileName = this->modelName + "_" + this->shooterName + "_" + std::to_string(this->projectileId);
        this->projectileSdfModel.SetName(projectileName);
        this->projectileSdfModel.SetRawPose(shooterPose * this->shooterOffset);
        Entity projectileModel = this->creator->CreateEntities(&this->projectileSdfModel);
        this->creator->SetParent(projectileModel, this->world);
        //update projectile,set velocity and create ContactSensorData
        math::Vector3d tmpVel(this->projectileVel, 0, 0);
        _ecm.CreateComponent(projectileModel, components::LinearVelocityCmd({ tmpVel }));
        Entity projectileLink = Model(projectileModel).Links(_ecm)[0];
        Entity projectileCollision = Link(projectileLink).Collisions(_ecm)[0];
        _ecm.CreateComponent(projectileCollision, components::ContactSensorData());
        //update queue and counters
        ProjectileInfo pInfo(projectileModel, projectileName, _info.simTime);
        this->spawnedProjectiles.push(pInfo);
        this->projectileId++;
        {
            std::lock_guard<std::mutex> lock(this->waitShootNumMutex);
            this->waitShootNum--;
        }
        this->currentTotalNum--;
    } else {
        //try to init the last projectile
        if (!this->spawnedProjectiles.empty()) {
            ProjectileInfo& pInfo = this->spawnedProjectiles.back();
            if (!pInfo.isInit) {
                //cancel velocity
                _ecm.RemoveComponent<components::LinearVelocityCmd>(pInfo.entity);
                pInfo.isInit = true;
            }
        }
    }
    //check cantact and time to delete projectiles
    if (!this->spawnedProjectiles.empty()) {
        ProjectileInfo& pInfo = this->spawnedProjectiles.front();
        if (pInfo.isInit) {
            Entity link = Model(pInfo.entity).Links(_ecm)[0];
            Entity collision = Link(link).Collisions(_ecm)[0];
            auto contacts = _ecm.Component<components::ContactSensorData>(collision)->Data();
            if (contacts.contact_size() > 0) {
                Entity collision1 = contacts.contact(0).collision1().id();
                Entity collision2 = contacts.contact(0).collision2().id();
                Entity targetCollision = (collision == collision1) ? collision2 : collision1;
                // TODO (maybe)
                // choose the best one according to some rules(for example,contain substring 'target') if the contact_size > 1 .
                // if (contacts.contact_size() > 1) {
                //     for (int i = 0; i < contacts.contact_size(); i++) {
                //         collision1 = contacts.contact(i).collision1().id();
                //         collision2 = contacts.contact(i).collision2().id();
                //         auto tmpCollision = (collision == collision1) ? collision2 : collision1;
                //         std::string name = *_ecm.ComponentData<components::Name>(tmpCollision);
                //         if(name.find("target")!=name.npos){
                //             targetCollision=tmpCollision;
                //             break;
                //         }
                //     }
                // }
                std::string result;
                auto name1 = scopedName(this->shooterLink, _ecm);
                auto name2 = scopedName(targetCollision, _ecm);
                result.insert(0, name2);
                result.insert(0, ">");
                result.insert(0, name1);
                // publish msg
                msgs::StringMsg msg;
                msg.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
                msg.set_data(result);
                this->attackPub.Publish(msg);
                //ignmsg << "ProjectileShooter contact_size: [" << contacts.contact_size() << "]" << std::endl;
                //ignmsg << "ProjectileShooter scop : [" << result << "]" << std::endl;
            }
            double t = std::chrono::duration_cast<std::chrono::milliseconds>(_info.simTime - pInfo.spawnTime).count();
            if (contacts.contact_size() > 0 || t > 4000) {
                this->creator->RequestRemoveEntity(pInfo.entity);
                this->spawnedProjectiles.pop();
            }
        }
    }
}

void ProjectileShooterPrivate::PostUpdate(const ignition::gazebo::UpdateInfo& /*_info*/,
    const ignition::gazebo::EntityComponentManager& /*_ecm*/)
{
}

/******************register*************************************************/
IGNITION_ADD_PLUGIN(ProjectileShooter,
    ignition::gazebo::System,
    ProjectileShooter::ISystemConfigure,
    ProjectileShooter::ISystemPreUpdate,
    ProjectileShooter::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ProjectileShooter, "ignition::gazebo::systems::ProjectileShooter")