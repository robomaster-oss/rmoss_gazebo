# rmoss_gz_base

## 简介

`rmoss_gz_base`为Gazebo Simulator中的机器人（主要为官方机器人`rmua19_standard_robot`）提供ROS接口，类似Robot Base功能（类似`rmoss_core/rmoss_base`），这样就可以通过ROS接口控制Gazebo仿真器中的机器人，也能通过ROS接口获得Gazebo仿真器中传感器的数据。同时为了实现高扩展性，将Gazebo执行器与传感器数据进行模块化。

Gazebo模块如下：

* `GzImu`：IMU传感器模块，并增加云台API，`get_yaw()`和`get_pitch()` 
* `GzJointEncoder`：关节编码器模块，并为云台增加额外API，`get_yaw()`和`get_pitch()` 
* `GzChassisCmd`：Gazebo底盘指令。
* `GzGimbalCmd`：Gazebo云台指令。

封装的ROS控制器如下：

* `ChassisController` : 底盘控制器，需要使用rmoss插件`MecanumDrive2` ，支持底盘速度控制，基于PID的底盘跟随云台控制。
  * 独立模式：速度控制。
  * 跟随模式：底盘方位角跟随云台，需要yaw轴电机的编码器数据。
* `GimbalController` : 云台控制器，需要使用官方Gazebo插件`JointController`，支持云台速度控制，基于PID的位置控制，同时，还支持获取云台当前角度，并以ROS topic形式发布。
  * 相对角控制
  * 增量角度控制
* `ShooterController` : 射击控制，需要使用rmoss插件`ProjectileShooter` 。

## 使用说明

使用Gz Module，Controller，Publisher等模块，构建自己的robot base节点（参考`Rmua19RobotBaseNode`）。

射击控制器

```c++
// 射击控制器
auto shooter_controller = std::make_shared<rmoss_gz_base::ShooterController>(
    ros_node, gz_node, "robot_base/shoot_cmd", gz_shooter_cmd_topic);
```

底盘控制器，使用ign模块，可实现多种组合。

```c++
// 需要先创建云台编码器（需要用到yaw轴电机位置，用于跟随云台）和底盘命令模块
auto gz_gimbal_encoder = std::make_shared<rmoss_gz_base::GzJointEncoder>(
    gz_node, gz_joint_state_topic);
auto gz_chassis_cmd = std::make_shared<rmoss_gz_base::GzChassisCmd>(gz_node, gz_chassis_cmd_topic);
// 创建底盘控制器
auto chassis_controller = std::make_shared<rmoss_gz_base::ChassisController>(
    ros_node, "robot_base/chassis_cmd", gz_chassis_cmd, gz_gimbal_encoder);
// 配置底盘控制器
chassis_controller->set_chassis_pid(chassis_pid_param);  // 设置自旋PID(用于跟随云台位置)
chassis_controller->set_control_mode(true);  // 设置跟随云台
```
