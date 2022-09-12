#!/usr/bin/python3
import sys

import rclpy
from rmoss_interfaces.msg import ShootCmd

def getShootCmdMsg(num,vel):
    msg = ShootCmd()
    msg.projectile_num=num
    msg.projectile_velocity=vel
    return msg

def main():
    rclpy.init()
    node = rclpy.create_node('test_shoot_cmd')
    pub = node.create_publisher(ShootCmd, 'shoot_cmd', 10)
    while True:
        print("\nEnter the hum of projectiles,default 1")
        try:
            num = int(input("num: "))
        except:
            break
        info=getShootCmdMsg(num,0.0)
        pub.publish(info)
        print("send--------------------------------\n")

if __name__ == '__main__':
    main()
