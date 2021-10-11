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
#include <ignition/common/Util.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Material.hh>

#include <ignition/gazebo/SdfEntityCreator.hh>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/Conversions.hh>

#include "LightIndicator.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;


sdf::Material GetMaterial(int state){
    sdf::Material m;
    ignition::math::Color color;
    if(state == 0){
        color.Set(1,1,1,1);
    }else if(state == 1){
        color.Set(1,0,0,1);
    }else if(state == 2){
        color.Set(0,0,1,1);
    }else{
        color.Set(1,1,0,1);
    }
    m.SetAmbient(color);
    m.SetDiffuse(color);
    m.SetSpecular(color);
    if(state == 0){
        color.Set(0.3,0.3,0.3,1);
        m.SetEmissive(color);
    }else{
        m.SetEmissive(color);
    }
    return m;
}


struct VisualEntityInfo {
    Entity entity;
    Entity parentEntity;
    sdf::Visual visualSdf;
    int state;
    VisualEntityInfo(Entity _entity,Entity parentEntity,sdf::Visual &_visualSdf,int _state)
        : entity(_entity)
        , parentEntity(parentEntity)
        , visualSdf(_visualSdf)
        , state(_state)
    {
    }
};

class ignition::gazebo::systems::LightIndicatorPrivate
{
public:
    void OnCmd(const ignition::msgs::Int32 &_msg);
    void Init(ignition::gazebo::EntityComponentManager &_ecm);
    void UpdateVisualEnitiies();

public:
    transport::Node node;
    //model
    Model model{kNullEntity};
    std::unique_ptr<SdfEntityCreator> creator { nullptr };
    sdf::Model modelSdf;
    std::vector<std::string> linkVisuals;
    std::vector<VisualEntityInfo> visualEntityInfos;
    bool isInit{false};
    bool isDone{true};
    // cmd
    // 0:white, 1:red light, 2:blue light, 3:yellow light
    int targetState;
    bool change{false};
    std::mutex targetMutex;
};

/******************implementation for LightIndicator************************/
LightIndicator::LightIndicator() : dataPtr(std::make_unique<LightIndicatorPrivate>())
{
}

void LightIndicator::Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager & _eventMgr)
{
    this->dataPtr->model = Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm))
    {
        ignerr << "LightIndicator plugin should be attached to a model entity. Failed to initialize." << std::endl;
        return;
    }
    // Get params from SDF
    std::string indicator_name = "indicator";
    if (_sdf->HasElement("indicator_name"))
    {
        indicator_name = _sdf->Get<std::string>("indicator_name");
    }
    if (_sdf->HasElement("initial_state"))
    {
        this->dataPtr->targetState = _sdf->Get<int>("initial_state");
        this->dataPtr->change = true;
    }
    // link_visual
    auto ptr = const_cast<sdf::Element *>(_sdf.get());
    sdf::ElementPtr sdfElem = ptr->GetElement("link_visual");
    while (sdfElem)
    {
        auto path = sdfElem->Get<std::string>();
        this->dataPtr->linkVisuals.push_back(std::move(path));
        sdfElem = sdfElem->GetNextElement("link_visual");
    }
    // creator
    this->dataPtr->creator = std::make_unique<SdfEntityCreator>(_ecm, _eventMgr);
    // Model Sdf
    this->dataPtr->modelSdf = _ecm.Component<components::ModelSdf>(_entity)->Data();
    // Subscribe to commands
    std::string topic{this->dataPtr->model.Name(_ecm) +"/"+indicator_name+ "/set_state"};
    this->dataPtr->node.Subscribe(topic, &LightIndicatorPrivate::OnCmd, this->dataPtr.get());
    ignmsg << "LightIndicator subscribing to int32 messages on [" << topic << "]" << std::endl;
}

void LightIndicator::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                             ignition::gazebo::EntityComponentManager &_ecm)
{
    if(_info.paused){
        return;
    }
    if(!this->dataPtr->isInit){
        this->dataPtr->Init(_ecm);
        this->dataPtr->isInit = true;
    }
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->targetMutex);
        //
        if(this->dataPtr->change){
            auto targetMaterial = GetMaterial(this->dataPtr->targetState);
            for(auto &info: this->dataPtr->visualEntityInfos){
                info.state = 0;
                info.visualSdf.SetMaterial(targetMaterial);
            }
            this->dataPtr->change = false; 
            this->dataPtr->isDone = false;
        }
    }
    if(!this->dataPtr->isDone){
        this->dataPtr->UpdateVisualEnitiies();
        // check
        bool flag = true;
        for(auto &info: this->dataPtr->visualEntityInfos){
            if(info.state<2){
                flag = false;
                break;
            }
        }
        this->dataPtr->isDone = flag;
    }
}


/******************implementation for LightIndicatorPrivate******************/
void LightIndicatorPrivate::OnCmd(const ignition::msgs::Int32 &_msg)
{
    std::lock_guard<std::mutex> lock(this->targetMutex);
    this->targetState = _msg.data();
    this->change = true;
}

void LightIndicatorPrivate::Init(ignition::gazebo::EntityComponentManager &_ecm){
    bool flag = false;
    for(auto linkVisual : this->linkVisuals){
        flag = false;
        auto v = common::split(linkVisual,"/");
        if(v.size() == 2){
            auto link = this->model.LinkByName(_ecm, v[0]);
            auto visual = _ecm.EntityByComponents(components::ParentEntity(link),components::Name(v[1]),components::Visual());
            if(visual != kNullEntity){
                sdf::Visual visualSdf = *(this->modelSdf.LinkByName(v[0])->VisualByName(v[1]));
                this->visualEntityInfos.emplace_back(visual,link,visualSdf,2);
                flag = true;
            }
        }
        if(!flag){
            ignerr << "LightIndicator: visual element of link [" << linkVisual << "] is invaild" << std::endl;
        }
    }
}

void LightIndicatorPrivate::UpdateVisualEnitiies(){
    for(auto &info: this->visualEntityInfos){
        if(info.state == 0){
            this->creator->RequestRemoveEntity(info.entity);
        }else if(info.state == 1){
            info.entity = this->creator->CreateEntities(&(info.visualSdf));
            this->creator->SetParent(info.entity , info.parentEntity);
        }
        if(info.state<2){
            info.state++;
        }  
    }
}

/******************register*************************************************/
IGNITION_ADD_PLUGIN(LightIndicator,
                    ignition::gazebo::System,
                    LightIndicator::ISystemConfigure,
                    LightIndicator::ISystemPreUpdate
                    )

IGNITION_ADD_PLUGIN_ALIAS(LightIndicator, "ignition::gazebo::systems::LightIndicator")