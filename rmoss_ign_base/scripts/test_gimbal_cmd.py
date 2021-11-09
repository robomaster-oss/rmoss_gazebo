#!/usr/bin/python3
import sys
if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

import rclpy
from rmoss_interfaces.msg import GimbalCmd

msg = """
This node takes keypresses from the keyboard and publishes them
as GimbalCmd messages.
---------------------------
contorl around:
        w    
   a    s    d
change  interval : '[' to decrease,  ']' to increase
---------------------------
CTRL-C to quit
"""

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def getGimbalContolMsg(pitch,yaw):
    control_info = GimbalCmd()
    control_info.position.yaw=yaw
    control_info.position.pitch=pitch
    return control_info

def clip(value,min_value,max_value):
    if(value<min_value):
        return min_value
    if(value>max_value):
        return max_value
    return value

def main():
    settings = saveTerminalSettings()
    rclpy.init()
    node = rclpy.create_node('control_gimbal_test')
    pub = node.create_publisher(GimbalCmd, 'gimbal_cmd', 10)
    print(msg)
    pitch=yaw=0.0
    da=0.05
    while True:
        key=getKey(settings)
        if key == 'w':
            pitch=pitch-da
        elif key == 's':
            pitch=pitch+da
        elif key == 'a':
            yaw=yaw+da
        elif key == 'd':
            yaw=yaw-da
        elif key == '[':
            da=da-0.01
        elif key == ']':
            da=da+0.01
        elif key == '\x03':
            break
        da=clip(da,0.01,0.2)
        info=getGimbalContolMsg(pitch,yaw)
        pub.publish(info)

if __name__ == '__main__':
    main()

