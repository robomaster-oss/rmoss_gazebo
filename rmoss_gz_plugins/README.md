# rmoss_gz_plugins

## 1.简介

rmoss_gz_plugins: 提供Gazebo Plugins，为RoboMaster Gazebo仿真提供插件支持。

* MecanumDrive: 麦克拉姆轮插件，实现底盘全向移动功能。
* ProjectileShooter：子弹发射插件，实现射击功能。
* LightBarController: 灯条控制器，可动态改变灯条发光颜色，支持`none`,`red`,`blue`,`yellow`几种模式。

## 2.使用说明

MecanumDrive2:

```xml
<plugin filename="MecanumDrive" name="ignition::gazebo::systems::MecanumDrive2">
    <chassis_link>chassis</chassis_link>
    <front_left_joint>front_left_wheel_joint</front_left_joint>
    <front_right_joint>front_right_wheel_joint</front_right_joint>
    <rear_left_joint>rear_left_wheel_joint</rear_left_joint>
    <rear_right_joint>rear_right_wheel_joint</rear_right_joint>
</plugin>
```

ProjectileShooter:

```xml
<plugin filename="ProjectileShooter" name="ignition::gazebo::systems::ProjectileShooter">
    <shooter_name>small_shooter</shooter_name>
    <shooter_link>speed_monitor</shooter_link>
    <shooter_offset>0.15 0 0 0 0 0</shooter_offset>
    <projectile_velocity>20</projectile_velocity>
    <projectile_num>10000</projectile_num>
    <projectile_uri>model://rm_fluorescent_projectile_17mm</projectile_uri>
</plugin>
```

LightBarController:

```xml
<plugin filename="LightBarController" name="ignition::gazebo::systems::LightBarController">
    <controller_name>color_controller</controller_name>
    <link_visual>armor_0/light_bar_visual</link_visual>
    <link_visual>armor_1/light_bar_visual</link_visual>
    <link_visual>armor_2/light_bar_visual</link_visual>
    <link_visual>armor_3/light_bar_visual</link_visual>
    <link_visual>light_indicator/light_bar_visual</link_visual>
    <link_visual>speed_monitor/light_bar_visual</link_visual>
    <initial_color>red</initial_color>
</plugin>
```
