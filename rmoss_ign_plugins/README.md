# rmoss_ign_plugins

## 1.简介

rmoss_ign_plugins: 提供Ignition Plugins，为RoboMaster Ignition仿真提供插件支持。

* MecanumDrive: 麦克拉姆轮插件，实现底盘全向移动功能。
* ProjectileShooter：子弹发射插件，实现射击功能。

## 2.使用说明

MecanumDrive：

```xml
<plugin filename="MecanumDrive" name="ignition::gazebo::systems::MecanumDrive">
            <chassis_link>chassis</chassis_link>
            <front_left_joint>front_left_wheel_joint</front_left_joint>
            <front_right_joint>front_right_wheel_joint</front_right_joint>
            <rear_left_joint>rear_left_wheel_joint</rear_left_joint>
            <rear_right_joint>rear_right_wheel_joint</rear_right_joint>
</plugin>
```

ProjectileShooter：

```xml
<plugin filename="ProjectileShooter" name="ignition::gazebo::systems::ProjectileShooter">
            <shooter_link>speed_monitor_17mm</shooter_link>
            <shooter_offset>0.15 0 0 0 0 0</shooter_offset>
            <shooter_name>small_shooter</shooter_name>
            <projectile_velocity>20</projectile_velocity>
    		<projectile_num>10000</projectile_num>
            <projectile_uri>model://rm_fluorescent_projectile_17mm</projectile_uri>
</plugin>
```

## 3.维护者及开源许可证

maintainer：Zhenpeng Ge,  zhenpeng.ge@qq.com

rmoss_ign_plugins is provided under MIT.