#!/usr/bin/env python3
# encoding: utf-8
# MentorPi_Acker 苹果定位 v3 (cmd_vel + odom + 时间戳补偿 + 深度距离)
#
# 改进:
#   1. odom 历史队列: 用图像拍摄时刻的 odom 位置, 而非检测到达时的位置
#   2. 深度距离优先: 用 depth distance 判断最近, 而非 bbox 面积
#   3. 两段式返回: 远处快, 近处慢, 避免冲过头
#
# 前提: 已启动
#   ros2 launch ros_robot_controller ros_robot_controller.launch.py
#   ros2 launch controller controller.launch.py
#   ros2 launch yolo_distance yolo_with_distance.launch.py use_distance:=True ...
#
# 用法:
#   python3 -u apple_spot_v3.py --distance 1.5 --speed 0.2
#   python3 -u apple_spot_v3.py --distance 1.5 --speed 0.2 --slow-speed 0.05 --slow-zone 0.15
from collections import deque
import time
import argparse
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, HistoryPolicy, ReliabilityPolicy,
                        qos_profile_sensor_data)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

try:
    from yolo_msgs.msg import DetectionArray
except ImportError:
    print('未找到 yolo_msgs, YOLO 检测将不可用', flush=True)
    DetectionArray = None


class AppleSpotNode(Node):
    def __init__(self, det_topic, class_name, min_score,
                 skip_frames=0, history_sec=5.0):
        super().__init__('apple_spot_v3')
        self.lock = threading.Lock()

        # odom
        self.x = 0.0
        self.got_odom = False
        self.odom_history = deque(maxlen=int(history_sec * 50))  # ~50Hz, 5秒

        # YOLO
        self.class_name = class_name
        self.min_score = min_score
        self.best_x = None        # 苹果最近时的 odom x (时间戳补偿后)
        self.best_dist = None     # 最近深度距离
        self.best_area = 0.0
        self.best_score = 0.0
        self.det_count = 0
        self.skip_frames = skip_frames
        self.frame_idx = 0

        # publishers / subscribers
        pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', pub_qos)
        self.create_subscription(Odometry, '/odom_raw', self.on_odom, 10)

        if DetectionArray is not None:
            q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(DetectionArray, det_topic, self.on_det, q)

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        with self.lock:
            self.x = x
            self.got_odom = True
            self.odom_history.append((t, x))

    def get_x_at_time(self, target_t):
        """查找最接近 target_t 的 odom x"""
        with self.lock:
            if not self.odom_history:
                return self.x
            best = min(self.odom_history, key=lambda p: abs(p[0] - target_t))
            return best[1]

    def on_det(self, msg):
        self.frame_idx += 1
        if self.skip_frames > 0 and self.frame_idx % (self.skip_frames + 1) != 0:
            return

        # 取图像拍摄时间戳
        image_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        for det in msg.detections:
            if det.class_name != self.class_name:
                continue
            if det.score < self.min_score:
                continue

            area = det.bbox.size.x * det.bbox.size.y
            p = det.bbox3d.center.position
            dist = (p.x**2 + p.y**2 + p.z**2)**0.5 if p.z > 0 else None

            # 用时间戳补偿: 查拍摄时刻的 odom 位置
            pose_x = self.get_x_at_time(image_t)

            with self.lock:
                self.det_count += 1
                # 优先用深度距离判断最近
                if dist is not None:
                    if self.best_dist is None or dist < self.best_dist:
                        self.best_x = pose_x
                        self.best_dist = dist
                        self.best_area = area
                        self.best_score = det.score
                        self.get_logger().info(
                            f'best(dist): x={pose_x:.3f} dist={dist:.2f} '
                            f'area={area:.0f} score={det.score:.2f}'
                        )
                elif self.best_dist is None:
                    # 没有深度距离时, 用面积 fallback
                    if self.best_x is None or area > self.best_area:
                        self.best_x = pose_x
                        self.best_area = area
                        self.best_score = det.score
                        self.get_logger().info(
                            f'best(area): x={pose_x:.3f} area={area:.0f} '
                            f'score={det.score:.2f}'
                        )

    def send_vel(self, linear_x):
        msg = Twist()
        msg.linear.x = linear_x
        self.cmd_pub.publish(msg)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main():
    parser = argparse.ArgumentParser(description='苹果定位 v3 (时间戳补偿 + 深度距离)')
    parser.add_argument('--distance', type=float, default=1.0,
                        help='最大前进距离(米)')
    parser.add_argument('--speed', type=float, default=0.2,
                        help='前进速度(m/s)')
    parser.add_argument('--slow-speed', type=float, default=0.05,
                        help='慢速倒车速度(m/s)')
    parser.add_argument('--slow-zone', type=float, default=0.15,
                        help='进入慢速区的距离(米)')
    parser.add_argument('--tolerance', type=float, default=0.02,
                        help='到位判定精度(米)')
    parser.add_argument('--class-name', default='apple', help='目标类别名')
    parser.add_argument('--min-score', type=float, default=0.5, help='最低置信度')
    parser.add_argument('--no-return', action='store_true', help='找到苹果后不返回')
    parser.add_argument('--det-topic', default='/yolo/detections_with_dist')
    parser.add_argument('--skip-frames', type=int, default=0,
                        help='每N帧检测一次(0=每帧)')
    parser.add_argument('--history-sec', type=float, default=5.0,
                        help='odom历史保留时长(秒)')
    args = parser.parse_args()

    rclpy.init()
    node = AppleSpotNode(args.det_topic, args.class_name, args.min_score,
                         skip_frames=args.skip_frames,
                         history_sec=args.history_sec)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # 等待 odom
    print('等待 odom 数据...', flush=True)
    for _ in range(100):
        if node.got_odom:
            break
        time.sleep(0.1)
    if not node.got_odom:
        print('未收到 odom, 检查 odom_publisher 是否运行', flush=True)
        rclpy.shutdown()
        return

    with node.lock:
        x_start = node.x
    print(f'初始 x = {x_start:.3f} m', flush=True)
    if args.skip_frames > 0:
        print(f'帧跳过: 每 {args.skip_frames + 1} 帧检测一次', flush=True)

    try:
        # ===== 前进搜索 =====
        print(f'\n前进搜索, 最远 {args.distance} m, 速度 {args.speed} m/s', flush=True)
        print(f'{"时间":>6} | {"x":>8} | {"dist":>8} | {"best_d":>8} | {"det":>4}')
        print('-' * 48)

        node.send_vel(args.speed)
        start = time.time()
        last_print = time.time()

        while True:
            with node.lock:
                x = node.x
                best_dist = node.best_dist
                det_count = node.det_count
            dist_from_start = abs(x - x_start)

            if time.time() - last_print >= 0.3:
                bd = f'{best_dist:.2f}' if best_dist else '?'
                print(f'{time.time()-start:6.2f} | {x:+8.3f} | {dist_from_start:8.3f} | '
                      f'{bd:>8} | {det_count:4d}', flush=True)
                last_print = time.time()

            if dist_from_start >= args.distance:
                print(f'\n到达最大距离 {args.distance:.2f} m', flush=True)
                break

            time.sleep(0.05)

        node.stop()
        time.sleep(0.5)

        # ===== 结果汇总 =====
        with node.lock:
            best_x = node.best_x
            best_dist = node.best_dist
            best_area = node.best_area
            best_score = node.best_score
            det_count = node.det_count
            x_end = node.x

        print(f'\n搜索结束, 共 {det_count} 帧检测', flush=True)
        print(f'最终 x = {x_end:+.3f} m', flush=True)

        if best_x is None:
            print(f'未检测到 {args.class_name}', flush=True)
            if not args.no_return:
                print('倒车回起点...', flush=True)
                node.send_vel(-args.speed)
                while True:
                    with node.lock:
                        x = node.x
                    if abs(x - x_start) < args.tolerance:
                        break
                    time.sleep(0.05)
                node.stop()
                print(f'已回到起点 x={x:.3f} m', flush=True)
        else:
            dist_to_apple = abs(x_end - best_x)
            print(f'\n找到苹果:', flush=True)
            print(f'  位置: x = {best_x:+.3f} m (时间戳补偿后)', flush=True)
            print(f'  深度距离: {round(best_dist, 2) if best_dist else "?"} m', flush=True)
            print(f'  面积: {best_area:.0f}', flush=True)
            print(f'  置信度: {best_score:.2f}', flush=True)
            print(f'  当前距苹果: {dist_to_apple:.3f} m', flush=True)

            if args.no_return:
                print('已停在当前位置', flush=True)
            else:
                # 两段式返回: 远处快, 近处慢
                print(f'\n返回苹果位置...', flush=True)

                # 阶段1: 快速接近
                while True:
                    with node.lock:
                        x = node.x
                    remaining = abs(x - best_x)
                    if remaining <= args.slow_zone:
                        break
                    node.send_vel(-args.speed)
                    time.sleep(0.05)

                # 阶段2: 慢速精确定位
                print(f'  进入慢速区 (剩 {abs(x - best_x):.3f} m)', flush=True)
                while True:
                    with node.lock:
                        x = node.x
                    remaining = abs(x - best_x)
                    if remaining < args.tolerance:
                        break
                    node.send_vel(-args.slow_speed)
                    time.sleep(0.05)

                node.stop()
                print(f'  已到达苹果位置 x={x:.3f} m (误差 {abs(x-best_x):.3f} m)', flush=True)

    except KeyboardInterrupt:
        print('\n手动停止', flush=True)
    finally:
        node.stop()
        print('已停止', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
