#!/usr/bin/python3
import sys

import rclpy
from rmoss_interfaces.msg import GimbalCmd

def getGimbalContolMsg(pitch,yaw):
    control_info = GimbalCmd()
    control_info.position.yaw=yaw
    control_info.position.pitch=pitch
    return control_info

def main():
    rclpy.init()
    node = rclpy.create_node('control_gimbal_test')
    pub = node.create_publisher(GimbalCmd, 'gimbal_cmd', 10)
    while True:
        print("\n[absolute mode],please intput tagret angle (float) ")
        try:
            pitch = float(input("pitch_angle: "))
            yaw = float(input("yaw_angle: "))
        except:
            break
        info=getGimbalContolMsg(pitch,yaw)
        pub.publish(info)
        print("send--------------------------------\n")

if __name__ == '__main__':
    main()
