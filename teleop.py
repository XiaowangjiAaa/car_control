#!/usr/bin/env python3
# encoding: utf-8
# MentorPi_Acker 键盘控制 (通过 ros2 topic pub, 与手柄同路径)
#
# 前提: bringup 节点需在运行
# 用法: source /opt/ros/humble/setup.zsh && python3 /home/ubuntu/car_control/teleop.py
import sys, os, tty, termios, subprocess, time, threading

TOPIC_VEL = '/controller/cmd_vel'
TOPIC_SERVO = '/ros_robot_controller/pwm_servo/set_state'

proc_vel = None

def publish_continuous(lx, az):
    global proc_vel
    if proc_vel:
        proc_vel.terminate()
        proc_vel.wait()
    msg = '{linear: {x: %.2f, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: %.2f}}' % (lx, az)
    proc_vel = subprocess.Popen(
        ['ros2', 'topic', 'pub', '-r', '20', TOPIC_VEL,
         'geometry_msgs/msg/Twist', msg],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

def stop():
    publish_continuous(0.0, 0.0)
    # 同时发舵机归中
    msg = '{state: [{id: [3], position: [1500]}], duration: 0.02}'
    subprocess.Popen(
        ['ros2', 'topic', 'pub', '--once', TOPIC_SERVO,
         'ros_robot_controller_msgs/msg/SetPWMServoState', msg],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def main():
    print('=== MentorPi Acker Keyboard Control (ROS2 cmd_vel) ===')
    print('w=forward  s=backward  a=left  d=right  SPACE=stop  q=quit')
    print('+=faster  -=slower  (current 0.20 m/s)')
    print('Lift car first to test direction')

    speed = 0.20
    try:
        while True:
            ch = getch()
            if ch in ('w', 'W'):
                publish_continuous(speed, 0.0)
                print('forward  %.2f m/s' % speed)
            elif ch in ('s', 'S'):
                publish_continuous(-speed, 0.0)
                print('backward  %.2f m/s' % speed)
            elif ch in ('a', 'A'):
                publish_continuous(speed, 0.5)
                print('left  %.2f + az0.5' % speed)
            elif ch in ('d', 'D'):
                publish_continuous(speed, -0.5)
                print('right  %.2f + az-0.5' % speed)
            elif ch == ' ':
                stop()
                print('STOP')
            elif ch == 'q':
                stop()
                print('quit')
                break
            elif ch in ('+', '='):
                speed = min(0.20, round(speed + 0.05, 2))
                print('speed: %.2f m/s' % speed)
            elif ch == '-':
                speed = max(0.05, round(speed - 0.05, 2))
                print('speed: %.2f m/s' % speed)
    finally:
        stop()

if __name__ == '__main__':
    main()
