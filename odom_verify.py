#!/usr/bin/env python3
# encoding: utf-8
#odom验证: 订阅/odom_raw + 同时驱动小车前进,观察x是否跟着变
import sys
import time
import threading

sys.path.insert(0, '/home/ubuntu/shared')
sys.path.insert(0, '/home/ubuntu/.local/lib/python3.10/site-packages')
import ros_robot_controller_sdk as rrc

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

MOTOR_LEFT_REAR = 2
MOTOR_RIGHT_REAR = 4


class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_test')
        self.lock = threading.Lock()
        self.x = 0.0
        self.vx = 0.0
        self.yaw = 0.0
        self.count = 0
        self.create_subscription(Odometry, '/odom_raw', self.on_odom, qos_profile_sensor_data)

    def on_odom(self, msg):
        with self.lock:
            self.x = msg.pose.pose.position.x
            self.yaw = msg.pose.pose.orientation.z
            self.vx = msg.twist.twist.linear.x
            self.count += 1


def main():
    rclpy.init()
    odom = OdomListener()
    thread = threading.Thread(target=rclpy.spin, args=(odom,), daemon=True)
    thread.start()

    print('等待 odom 数据...', flush=True)
    for _ in range(50):
        if odom.count > 0:
            break
        time.sleep(0.1)

    board = rrc.Board()
    print('串口已打开', flush=True)

    speed = 0.5
    duration = 3.0
    sign = -1  # M2负号=前进

    try:
        print(f'\n前进 {duration} 秒, speed={speed}', flush=True)
        print(f'{"时间":>6} | {"x":>8} | {"vx":>8} | {"odom帧数":>8}')
        print('-' * 40)

        board.set_motor_speed([
            [1, 0.0],
            [MOTOR_LEFT_REAR, sign * speed],
            [3, 0.0],
            [MOTOR_RIGHT_REAR, speed],
        ])

        start = time.time()
        while time.time() - start < duration:
            with odom.lock:
                x, vx, count = odom.x, odom.vx, odom.count
            t = time.time() - start
            print(f'{t:6.2f} | {x:+8.3f} | {vx:+8.3f} | {count:8d}', flush=True)
            time.sleep(0.2)

        board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        print(f'\n停止, 最终 x = {odom.x:+.3f} m', flush=True)

        # 观察停止后x是否稳定
        print('\n停止后观察3秒...', flush=True)
        for i in range(15):
            with odom.lock:
                x, vx = odom.x, odom.vx
            print(f'  t={i*0.2:.1f}s x={x:+.3f} vx={vx:+.3f}', flush=True)
            time.sleep(0.2)

    finally:
        board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        print('已停止', flush=True)
        odom.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
