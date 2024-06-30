# rmoss_gazebo

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/robomaster-oss/rmoss_gazebo/actions/workflows/ci.yml/badge.svg?branch=humble)](https://github.com/robomaster-oss/rmoss_gazebo/actions/workflows/ci.yml)

![](rmoss_bg.png)
RoboMasterOSS是一个面向RoboMaster的开源软件栈项目，目的是为RoboMaster机器人软件开发提供了一个快速的，灵活的开发工具，支持算法原型研究和robomaster比赛应用开发。

> [RoboMaster竞赛](https://www.robomaster.com/)，全称为`全国大学生机器人大赛RoboMaster机甲大师赛` 。
>
> - 全国大学生机器人[RoboMaster](https://www.robomaster.com/)大赛，是一个涉及“机器视觉”、“嵌入式系统设计”、“机械控制”、“人机交互”等众多机器人相关技术学科的机器人比赛。
> - 在RoboMaster 2019赛季中，参赛队伍需自主研发不同种类和功能的机器人，在指定的比赛场地内进行战术对抗，通过操控机器人发射弹丸攻击敌方机器人和基地。每局比赛7分钟，比赛结束时，基地剩余血量高的一方获得比赛胜利。
>
> 更多详情参考官网：[www.robomaster.com](https://www.robomaster.com/)

rmoss_gazebo是RoboMaster OSS中的基础项目，为RoboMaster提供Gazebo仿真支持，主要提供Gazebo插件，相关机器人模型资源等。

## 主要模块

|            模块             |                           功能说明                           |
| :-------------------------: | :----------------------------------------------------------: |
|     `rmoss_gz_plugins`     |        RoboMaster相关Gazebo Simulator插件。                  |
|      `rmoss_gz_base`       | Gazebo机器人基本接口(对应`rmoss_base`), 模拟MCU部分功能       |
|      `rmoss_gz_cam`        | Gazebo相机接口(对应`rmoss_cam`)                              |
|    `rmoss_gz_resources`    | RoboMaster相关核心SDF模型资源，官方机器人模型和核心场地模型。 |

* `rmoss_gz_resources` 主要包含资源文件，体积较大，单独成库。

## 使用说明

环境依赖

* ROS2版本: `Humble`.
* Gazebo仿真器版本（新版）: ` Fortress`.
* RMOSS项目依赖（需要源码编译）: [rmoss_interfaces](https://github.com/robomaster-oss/rmoss_interfaces) ,[rmoss_gz_resources](https://github.com/robomaster-oss/rmoss_gz_resources.git).

> 经典Gazebo(数字版本) 与 新版Gazebo (字母版本，也叫Ignition，目前还在迁移过程中)  差别较大，该项目基于新版Gazebo，命名空间将会进行逐步调整。

环境配置

```bash
# install gazebo dependencies
sudo apt-get install ignition-fortress libignition-cmake2-dev ros-humble-ros-gz
# cd ros2 workspaces src
git clone https://github.com/robomaster-oss/rmoss_gazebo.git -b humble
git clone https://github.com/robomaster-oss/rmoss_interfaces.git -b humble
git clone https://github.com/robomaster-oss/rmoss_gz_resources.git -b humble --depth=1
# cd ros2 workspaces
cd ..
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
colcon build
```

* 相关功能包使用详见相应package的README.md

## RMOSS Gazebo设计

* 详见[RMOSS Gazebo设计模式](https://robomaster-oss.github.io/rmoss_tutorials/#/design/rmoss_ign_design)

## 维护者及开源许可证

Maintainer : Zhenpeng Ge,  zhenpeng.ge@qq.com

rmoss_gazebo is provided under Apache License 2.0.

