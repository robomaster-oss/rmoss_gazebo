# rmoss_ign_base

## 简介

`rmoss_ign_base`为Ignition Gazebo Simulator中的机器人（主要为官方机器人`rmua19_standard_robot`）提供ROS接口，类似Robot Base功能（类似`rmoss_core/rmoss_base`），这样就可以通过ROS接口控制Ignition Gazebo仿真器中的机器人，也能通过ROS接口获得Ignition Gazebo仿真器中传感器的数据。包含以下两类模型：

* `Controller` : 主要面向机器人控制器，为ROS->Ignition Gazebo的通信，将ROS命令发送给Ignition Gazebo Simulator中的机器人，控制机器人的执行机构，同时实现基于PID的控制器，可以在ROS中实现相关控制任务，如底盘跟随云台控制器。
* `Publisher` : 主要面向机器人传感器，为Ignition Gazebo->ROS的通信，将Ignition Gazebo Simulator中的机器人的传感器数据以Topic的形式发送到ROS中，可以被上层应用，如IMU数据。

`Controller`种类：

* ChassisController：支持速度控制，基于PID的底盘跟随云台控制，需要使用rmoss插件`MecanumDrive2` 。
* GimbalController：支持速度控制，基于PID的位置控制，需要使用官方Ignition Gazebo插件`JointController`。
* ShooterController：射击控制，需要使用rmoss插件`ProjectileShooter` 。

`Publisher` 种类：

* GimbalStatePublisher：云台数据发布，采用`IgnImu`模块，获取云台IMU角度，然后发布云台角度控制。
* OdometryPublisher：里程计数据发布

## 使用说明

使用Ign，Controller，Publisher等模块，构建自己的robot base节点（参考`Rmua19RobotBaseNode`）。

射击控制器

```c++
// 射击控制器
auto shooter_controller = std::make_shared<rmoss_ign_base::ShooterController>(
    ros_node, ign_node, "robot_base/shoot_cmd", ign_shooter_cmd_topic);
```

底盘控制器，使用ign模块，可实现多种组合。

```c++
// 需要先创建云台编码器（需要用到yaw轴电机位置，用于跟随云台）和底盘命令模块
auto ign_gimbal_encoder = std::make_shared<rmoss_ign_base::IgnJointEncoder>(
    ign_node, ign_joint_state_topic);
auto ign_chassis_cmd = std::make_shared<rmoss_ign_base::IgnChassisCmd>(ign_node, ign_chassis_cmd_topic);
// 创建底盘控制器
auto chassis_controller = std::make_shared<rmoss_ign_base::ChassisController>(
    ros_node, "robot_base/chassis_cmd", ign_chassis_cmd, ign_gimbal_encoder);
// 配置底盘控制器
chassis_controller->set_chassis_pid(chassis_pid_param);  // 设置自旋PID(用于跟随云台位置)
chassis_controller->set_control_mode(true);  // 设置跟随云台
```


## Ign模块解耦

为了实现云台与底盘控制器高扩展性，将ign执行器与传感器数据进行模块化，有如下模块：

* IgnImu：IMU传感器模块，并增加云台API，`get_yaw()`和`get_pitch()` 
* IgnJointEncoder：关节编码器模块，并为云台增加额外API，`get_yaw()`和`get_pitch()` 
* IgnChassisCmd：Ignition底盘指令
* IgnGimbalCmd：Ignition云台指令

> 传感器模块通过订阅Ignition topic，其数据与Ignition Gazbeo中保持同步。

ChassisController：底盘自旋PID控制
* 实现了底盘的独立控制模式和跟随模式。
  * 独立模式：速度控制。
  * 跟随模式：底盘方位角跟随云台，需要yaw轴电机的编码器数据。

> 后期考虑支持扭腰模式和陀螺模式

GimbalController：云台位置PID控制
* 采用IMU传感器作为角度传感器，被控对象为云台速度，实现云台角度控制。

> 若`JointController`插件采用PID控制，则云台的控制可以看作双环PID控制，为简化PID调参，`JointController`插件采用直接速度控制，而不使用力控方式。
