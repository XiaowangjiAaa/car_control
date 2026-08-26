#!/usr/bin/env python3
# encoding: utf-8
# 通过cmd_vel前进,同时监听/odom_raw,验证x是否跟着变
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class OdomTest(Node):
    def __init__(self):
        super().__init__('odom_cmdvel_test')
        self.lock = threading.Lock()
        self.x = 0.0
        self.vx = 0.0
        self.count = 0

        pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Twist, '/controller/cmd_vel', pub_qos)
        self.create_subscription(Odometry, '/odom_raw', self.on_odom, 10)

    def on_odom(self, msg):
        with self.lock:
            self.x = msg.pose.pose.position.x
            self.vx = msg.twist.twist.linear.x
            self.count += 1

    def get(self):
        with self.lock:
            return self.x, self.vx, self.count


def main():
    rclpy.init()
    node = OdomTest()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    print('等待 odom 数据...', flush=True)
    for _ in range(50):
        if node.get()[2] > 0:
            break
        time.sleep(0.1)

    x0, _, _ = node.get()
    print(f'初始 x = {x0:.3f} m\n', flush=True)

    speed = 0.2
    duration = 3.0

    # 发cmd_vel前进
    msg = Twist()
    msg.linear.x = speed
    print(f'发送 cmd_vel: linear.x = {speed} m/s, 持续 {duration} 秒', flush=True)
    print(f'{"时间":>6} | {"x":>8} | {"vx":>8} | {"帧数":>6}')
    print('-' * 38)

    node.pub.publish(msg)
    start = time.time()

    try:
        while time.time() - start < duration:
            x, vx, count = node.get()
            t = time.time() - start
            print(f'{t:6.2f} | {x:+8.3f} | {vx:+8.3f} | {count:6d}', flush=True)
            time.sleep(0.2)

        # 停车
        node.pub.publish(Twist())
        time.sleep(0.5)

        x_end, _, _ = node.get()
        print(f'\n停止, 最终 x = {x_end:+.3f} m (移动了 {x_end - x0:+.3f} m)', flush=True)

        # 停止后观察
        print('\n停止后观察2秒...', flush=True)
        for i in range(10):
            x, vx, _ = node.get()
            print(f'  x={x:+.3f} vx={vx:+.3f}', flush=True)
            time.sleep(0.2)

    finally:
        node.pub.publish(Twist())
        print('已停止', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
