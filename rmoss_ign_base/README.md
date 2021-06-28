# rmoss_ign_base

## 1.简介

`rmoss_ign_base`为Ignition Gazebo Simulator中的机器人（主要为官方机器人`rmua19_standard_robot`）提供Robot Base功能（类似`rmoss_core/rm_base`），实现了相关Ignition-ROS2 bridge，以及一些简单的PID控制器，目前包含以下几个功能：

* simpler_controller：负责将ROS2控制消息转发到Ignition Gazebo中（Ignition-ROS2 bridge），实际控制器由Ignition Gazebo中的Plugin实现。
* advanced_controller：采用PID控制器，对Igntion Gazebo中的机器人进行闭环控制，目前包括云台控制和底盘控制器
* gimbal_state_publisher：负责将Ignition Gazebo传感器消息转发到ROS2中（Ignition-ROS2 bridge）

## 2.文件说明

**ign封装模块**

* IgnImu：IMU传感器模块，并增加云台API，`getYaw()`和`getPitch()` 
* IgnJointEncoder：关节编码器模块，并为云台增加额外API，`getYaw()`和`getPitch()` 
* IgnChassisCmd：Ignition底盘指令
* IgnGimbalCmd：Ignition云台指令

> 传感器模块通过订阅Ignition topic，其数据与Ignition Gazbeo中保持同步。

**simpler_controller系列**

* SimpleChassisController：底盘控制（`rmoss_interfaces::msg::ChassisCmd`-> `ignition::msgs::Twist`）
  * 适用于使用rmoss插件`MecanumDrive2`的情况。
* SimpleGimbalController：云台控制（`rmoss_interfaces::msg::GimbalCmd` -> `ignition::msgs::Double`）
  * 支持速度控制命令或者位置控制命令。
  * 适用于云台使用官方Ignition Gazebo插件`JointPositionController` / `JointController`的情况。
* SimpleShooterController：射击控制（`rmoss_interfaces::msg::ShootCmd` -> `ignition::msgs::Double`）
  * 适用于使用rmoss插件`ProjectileShooter` 的情况。

**advanced_controller系列**

* AdvancedChassisController：底盘自旋PID控制
  * 实现了底盘的独立控制模式和跟随模式。
    * 独立模式：等价于SimpleChassisController。
    * 跟随模式：底盘方位角跟随云台，需要yaw轴电机的编码器数据。
  * 适用于使用rmoss插件`MecanumDrive2`的情况。

> 后期考虑支持扭腰模式和陀螺模式

* AdvancedGimbalController：云台PID控制
  * 采用IMU传感器作为角度传感器，被控对象为速度，实现云台角度控制。
  * 适用于云台使用官方Ignition Gazebo插件`JointController`的情况。

> 若`JointController`插件采用PID控制，则云台的控制可以看作双环PID控制，为简化PID调参，`JointController`插件不使用力控方式。

**其它**

* GimbalStatePublisher：云台信息发布
  * 采用`IgnImu`模块，获取云台IMU角度，然后发布云台角度控制。

> 由于目前云台考虑IMU角度PID控制，所以这里的云台角度数据采用IMU数据（`IgnImu`），而不是采用编码器数据（`IgnJointEncoder`）。

## 3.使用说明

**launch方式**

* 在launch中，使用nodes目录中的node，通过ros parameter传入相应参数。

**C++方式**

* 编写自己的node，使用IgnImu，Controller等模块，构建自己的robot base节点。

>  推荐使用C++，因为为每一个控制器创建一个node节点是冗余的，可以将多个控制器创建在一个node中，提高性能。

## 4.维护者及开源许可证

maintainer：Zhenpeng Ge,  zhenpeng.ge@qq.com

rmoss_ign_base is provided under MIT.