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

#include <mutex>
#include <ignition/common/Util.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/Conversions.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/PID.hh>

#include "MecanumDrive2.hh"

#define WHEEL_NUM 4
using namespace ignition;
using namespace gazebo;
using namespace systems;

const std::string kSdfElemJointNames[WHEEL_NUM] = {"front_right_joint", "front_left_joint", "rear_right_joint", "rear_left_joint"};

class ignition::gazebo::systems::MecanumDrive2Private
{
public:
    // Indicates joint/link of which wheel
    enum
    {
        FRONT_RIGHT = 0,
        FRONT_LEFT = 1,
        REAR_RIGHT = 2,
        REAR_LEFT = 3
    };

public:
    void OnCmdVel(const ignition::msgs::Twist &_msg);

    void UpdateOdometry(const ignition::gazebo::UpdateInfo &_info, const ignition::gazebo::EntityComponentManager &_ecm);

public:
    transport::Node node;
    //model
    Model model{kNullEntity};
    //chassis link
    std::string chassisLinkName;
    Entity chassisLink{kNullEntity};
    //wheel joint
    std::string wheelJointNames[WHEEL_NUM];
    Entity wheelJoints[WHEEL_NUM];
    Entity wheelLinks[WHEEL_NUM];
    //pid
    ignition::math::PID xPid;
    ignition::math::PID yPid;
    ignition::math::PID wPid;
    //for Odometry
    bool initFlag = false;
    std::string odomFrameId;
    std::string odomChildFrameId;
    ignition::math::Pose3d initPose;
    ignition::math::Pose3d lastPose;
    transport::Node::Publisher odomPub;
    //velocity cmd
    msgs::Twist targetVel;
    std::mutex targetVelMutex;
};

/******************implementation for MecanumDrive2************************/
MecanumDrive2::MecanumDrive2() : dataPtr(std::make_unique<MecanumDrive2Private>())
{
}

void MecanumDrive2::Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager & /*_eventMgr*/)
{
    this->dataPtr->model = Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm))
    {
        ignerr << "MecanumDrive2 plugin should be attached to a model entity. Failed to initialize." << std::endl;
        return;
    }
    // Get params from SDF
    // Get chassis link
    this->dataPtr->chassisLinkName = _sdf->Get<std::string>("chassis_link");
    this->dataPtr->chassisLink = this->dataPtr->model.LinkByName(_ecm, this->dataPtr->chassisLinkName);
    if (this->dataPtr->chassisLink == kNullEntity)
    {
        ignerr << "chassis link with name[" << this->dataPtr->chassisLinkName << "] not found. " << std::endl;
        return;
    }
    //Get joints and links of wheel
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        this->dataPtr->wheelJointNames[i] = _sdf->Get<std::string>(kSdfElemJointNames[i]);
        this->dataPtr->wheelJoints[i] = this->dataPtr->model.JointByName(_ecm, this->dataPtr->wheelJointNames[i]);
        if (this->dataPtr->wheelJoints[i] == kNullEntity)
        {
            ignerr << "wheel joint with name[" << this->dataPtr->wheelJointNames[i] << "] not found. " << std::endl;
            return;
        }
    }
    // Subscribe to commands
    std::string topic{this->dataPtr->model.Name(_ecm) + "/cmd_vel"};
    this->dataPtr->node.Subscribe(topic, &MecanumDrive2Private::OnCmdVel, this->dataPtr.get());
    ignmsg << "MecanumDrive2 subscribing to twist messages on [" << topic << "]" << std::endl;
    //publisher of odometry
    std::string odomTopic{this->dataPtr->model.Name(_ecm) + "/odometry"};
    this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(odomTopic);
    this->dataPtr->odomFrameId=this->dataPtr->model.Name(_ecm) + "/odom" ;
    this->dataPtr->odomChildFrameId = this->dataPtr->model.Name(_ecm) + "/" + ignition::common::replaceAll(this->dataPtr->chassisLinkName, "::", "/");
    //init PID
    this->dataPtr->xPid.Init(100, 0, 0, 0, 0, 100, -100, 0);
    this->dataPtr->yPid.Init(500, 0, 0, 0, 0, 200, -200, 0);
    this->dataPtr->wPid.Init(200, 0, 0, 0, 0, 100, -100, 0);
}

void MecanumDrive2::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                             ignition::gazebo::EntityComponentManager &_ecm)
{
    //control for chassis
    Link chassisLink(this->dataPtr->chassisLink);
    if (!_ecm.Component<components::WorldPose>(this->dataPtr->chassisLink))
    {
        _ecm.CreateComponent(this->dataPtr->chassisLink, components::WorldPose());
    }
    if (!_ecm.Component<components::LinearVelocity>(this->dataPtr->chassisLink))
    {
        _ecm.CreateComponent(this->dataPtr->chassisLink, components::LinearVelocity());
    }
    if (!_ecm.Component<components::AngularVelocity>(this->dataPtr->chassisLink))
    {
        _ecm.CreateComponent(this->dataPtr->chassisLink, components::AngularVelocity());
    }
    //mutex for targetVel
    msgs::Twist targetVel;
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->targetVelMutex);
        targetVel = this->dataPtr->targetVel;
    }
    //current state
    const auto chassisPose = _ecm.Component<components::WorldPose>(this->dataPtr->chassisLink)->Data();
    const auto linearVel = _ecm.Component<components::LinearVelocity>(this->dataPtr->chassisLink)->Data();
    const auto angularVel = _ecm.Component<components::AngularVelocity>(this->dataPtr->chassisLink)->Data();
    if (!this->dataPtr->initFlag)
    {
        this->dataPtr->initPose = chassisPose;
        this->dataPtr->initFlag = true;
    }
    //for linear velocity control
    double xErr = linearVel.X() - targetVel.linear().x();
    double xCmd = this->dataPtr->xPid.Update(xErr, _info.dt);
    double yErr = linearVel.Y() - targetVel.linear().y();
    double yCmd = this->dataPtr->yPid.Update(yErr, _info.dt);
    //for angular velocity control
    double wErr = angularVel.Z() - targetVel.angular().z();
    double wCmd = this->dataPtr->wPid.Update(wErr, _info.dt);
    //force and torque on chassis link frame
    math::Vector3d tmpForce(xCmd, yCmd, 0);
    math::Vector3d tmpTorque(0, 0, wCmd);
    // transform to world frame
    auto force = chassisPose.Rot().RotateVector(tmpForce);
    auto torque = chassisPose.Rot().RotateVector(tmpTorque);
    // ignmsg << "MecanumDrive2 (force,torque):[" << force << "], [" << torque << "]" << std::endl;
    // Apply the wrench
    chassisLink.AddWorldWrench(_ecm, force, torque);
}
void MecanumDrive2::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                              const ignition::gazebo::EntityComponentManager &_ecm)
{

    // 1.check collsion  of wheel's link and set the wheel's state true if the wheel contacts with ground plane, and
    // then can compute force and torque based wheel states. (TODO)
    // 2.for odometer
    this->dataPtr->UpdateOdometry(_info, _ecm);
}

/******************implementation for MecanumDrive2Private******************/

void MecanumDrive2Private::OnCmdVel(const ignition::msgs::Twist &_msg)
{
    std::lock_guard<std::mutex> lock(this->targetVelMutex);
    this->targetVel = _msg;
    //ignmsg << "MecanumDrive2 msg x: [" << _msg.linear().x() << "]" << std::endl;
}

void MecanumDrive2Private::UpdateOdometry(const ignition::gazebo::UpdateInfo &_info,
                                         const ignition::gazebo::EntityComponentManager &_ecm)
{
    //get pose and velocity of chassis
    Link chassisLink(this->chassisLink);
    const auto chassisPose = _ecm.Component<components::WorldPose>(this->chassisLink)->Data();
    const auto linearVel = _ecm.Component<components::LinearVelocity>(this->chassisLink)->Data();
    const auto angularVel = _ecm.Component<components::AngularVelocity>(this->chassisLink)->Data();
    auto diffPose = chassisPose - initPose;
    // Construct the odometry message and publish it.
    msgs::Odometry msg;
    msg.mutable_pose()->mutable_position()->set_x(chassisPose.X());
    msg.mutable_pose()->mutable_position()->set_y(chassisPose.Y());
    math::Quaterniond orientation(0, 0, chassisPose.Yaw());
    msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);

    msg.mutable_twist()->mutable_linear()->set_x(linearVel.X());
    msg.mutable_twist()->mutable_linear()->set_y(linearVel.Y());
    msg.mutable_twist()->mutable_angular()->set_z(angularVel.Z());
    // Set the time stamp in the header
    msg.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
    // Set the frame id.
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->odomFrameId);
    auto childFrame = msg.mutable_header()->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(this->odomChildFrameId);
    // Publish the message
    this->odomPub.Publish(msg);
}

/******************register*************************************************/
IGNITION_ADD_PLUGIN(MecanumDrive2,
                    ignition::gazebo::System,
                    MecanumDrive2::ISystemConfigure,
                    MecanumDrive2::ISystemPreUpdate,
                    MecanumDrive2::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MecanumDrive2, "ignition::gazebo::systems::MecanumDrive2")