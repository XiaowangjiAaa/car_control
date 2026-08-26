#!/usr/bin/env python3
# encoding: utf-8
# MentorPi_Acker 基于 IMU 的直行 + 原路返回
# 用法(容器内, 需已启动 realsense, ros_robot_controller 服务需停止):
#   python3 -u drive_straight.py --forward 2.5 --back 2.5 --speed 0.3
import sys
import os
import time
import argparse
import threading

sys.path.insert(0, '/home/ubuntu/shared')
import ros_robot_controller_sdk as rrc

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

MOTOR_TYPE_ACKER = 0x02
STEER_SERVO_ID = 2
STEER_CENTER = 1500
# 实测: M2负号=前进, M4正号=前进
MOTOR_LEFT_REAR = 2
MOTOR_RIGHT_REAR = 4
DEG = 180.0 / 3.14159265358979

class ImuReader(Node):
    def __init__(self, topic):
        super().__init__('imu_reader')
        self.lock = threading.Lock()
        self.gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.accel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.ts = 0.0
        self.create_subscription(Imu, topic, self.on_imu, qos_profile_sensor_data)

    def on_imu(self, msg):
        with self.lock:
            self.gyro['x'] = msg.angular_velocity.x
            self.gyro['y'] = msg.angular_velocity.y
            self.gyro['z'] = msg.angular_velocity.z
            self.accel['x'] = msg.linear_acceleration.x
            self.accel['y'] = msg.linear_acceleration.y
            self.accel['z'] = msg.linear_acceleration.z
            self.ts = time.time()

class YawEstimator:
    def __init__(self, imu, yaw_sign):
        self.imu = imu
        self.yaw_sign = yaw_sign
        self.bias_y = 0.0
        self.yaw = 0.0
        self.last_ts = None

    def calibrate(self, duration):
        print('请保持小车完全静止 {} 秒进行陀螺零偏标定...'.format(duration), flush=True)
        samples = []
        start = time.time()
        while time.time() - start < duration:
            with self.imu.lock:
                samples.append(self.imu.gyro['y'])
            time.sleep(0.05)
        self.bias_y = sum(samples) / len(samples)
        print('陀螺零偏 y = {:.4f}'.format(self.bias_y), flush=True)

    def update(self):
        now = time.time()
        with self.imu.lock:
            gy = self.imu.gyro['y']
            got = self.imu.ts > 0
        if not got:
            return
        if self.last_ts is not None:
            dt = now - self.last_ts
            if dt > 0.2:
                dt = 0.0
            self.yaw += self.yaw_sign * (gy - self.bias_y) * dt
        self.last_ts = now

    def reset(self):
        self.yaw = 0.0
        self.last_ts = None

def drive(board, est, duration, direction, speed, kp, steer_sign, step=0.02):
    start = time.time()
    last_steer = 0.0
    last_print = 0.0
    while time.time() - start < duration:
        est.update()
        err_deg = est.yaw * DEG
        if time.time() - last_steer >= 0.1:
            pulse = int(STEER_CENTER + steer_sign * kp * err_deg)
            pulse = max(1000, min(2000, pulse))
            board.pwm_servo_set_position(0.1, [[STEER_SERVO_ID, pulse]])
            last_steer = time.time()
        if direction == 'forward':
            left, right = -speed, speed   # M2负号=前进, M4正号=前进
        else:
            left, right = speed, -speed
        board.set_motor_speed([
            [1, 0],
            [MOTOR_LEFT_REAR, left],
            [3, 0],
            [MOTOR_RIGHT_REAR, right],
        ])
        if time.time() - last_print >= 0.5:
            print('[{}] t={:4.1f}s yaw={:+6.1f} deg'.format(direction, time.time() - start, err_deg), flush=True)
            last_print = time.time()
        time.sleep(step)
    board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
    print('{} 结束, 最终 yaw = {:+6.1f} deg'.format(direction, est.yaw * DEG), flush=True)

def main():
    parser = argparse.ArgumentParser(description='MentorPi_Acker IMU 直行/返回')
    parser.add_argument('--forward', type=float, default=2.0, help='前进时长(秒)')
    parser.add_argument('--back', type=float, default=0.0, help='返回时长(秒), 0 则不返回')
    parser.add_argument('--speed', type=float, default=0.3, help='速度 0~1')
    parser.add_argument('--calib', type=float, default=2.0, help='静止标定时长(秒)')
    parser.add_argument('--kp', type=float, default=5.0, help='转向增益(脉冲/度)')
    parser.add_argument('--steer-sign', type=int, default=1, help='转向符号 1/-1')
    parser.add_argument('--yaw-sign', type=int, default=1, help='yaw 符号 1/-1')
    parser.add_argument('--topic', default='/camera/camera/imu')
    args = parser.parse_args()

    if args.back == 0:
        args.back = args.forward

    rclpy.init()
    imu = ImuReader(args.topic)
    thread = threading.Thread(target=rclpy.spin, args=(imu,), daemon=True)
    thread.start()

    print('等待 IMU 数据...', flush=True)
    for _ in range(200):
        if imu.ts > 0:
            break
        time.sleep(0.05)
    if imu.ts == 0:
        print('未收到 IMU 数据, 检查 realsense 是否运行', flush=True)
        rclpy.shutdown()
        return

    est = YawEstimator(imu, args.yaw_sign)
    est.calibrate(args.calib)
    est.reset()

    print('打开串口 /dev/ttyACM0 ...', flush=True)
    board = rrc.Board()
    board.set_motor_type(MOTOR_TYPE_ACKER)
    board.pwm_servo_set_position(0.5, [[STEER_SERVO_ID, STEER_CENTER]])
    time.sleep(0.5)

    try:
        print('前进 {} 秒, speed={}'.format(args.forward, args.speed), flush=True)
        drive(board, est, args.forward, 'forward', args.speed, args.kp, args.steer_sign)
        time.sleep(0.5)
        if args.back > 0:
            print('返回 {} 秒 ...'.format(args.back), flush=True)
            drive(board, est, args.back, 'backward', args.speed, args.kp, args.steer_sign)
        print('完成, 回到起始位置(理论)', flush=True)
    except KeyboardInterrupt:
        print('\n手动停止', flush=True)
    finally:
        board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        board.pwm_servo_set_position(0.5, [[STEER_SERVO_ID, STEER_CENTER]])
        print('已停止', flush=True)
        imu.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
