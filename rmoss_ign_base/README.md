# rmoss_ign_robot_base

## 1.简介

提供Ignition Gazebo Simulator中的Robot Base模块（主要为官方standard robot提供支持），目前包含以下几个功能：

* SimpleChassisController：底盘控制（`rmoss_interfaces::msg::ChassisCmd`-> `ignition::msgs::Twist`），适用于使用rmoss插件`MecanumDrive`的情况。
* SimpleGimbalController：云台控制（`rmoss_interfaces::msg::GimbalCmd` -> 2 `ignition::msgs::Double`），适用于使用官方插件`JointPositionController`的情况。
* GimbalStatePublisher：云台信息发布（`ignition::msgs::Model`->`rmoss_interfaces::msg::Gimbal` ），适用于使用官方插件`JointStatePublisher` 的情况。
* ShooterSimpleController：射击控制（`rmoss_interfaces::msg::ShootCmd` -> `ignition::msgs::Double`），适用于使用rmoss插件`ProjectileShooter` 的情况。

## 2.文件说明

* TODO

## 3.使用说明

* TODO

## 4.维护者及开源许可证

maintainer：Zhenpeng Ge,  zhenpeng.ge@qq.com

rmoss_ign_robot_base is provided under MIT.