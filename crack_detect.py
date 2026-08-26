#!/usr/bin/env python3
# encoding: utf-8
# 裂缝检测 + 定位返回 (cmd_vel + odom + 时间戳补偿)
#
# 流程:
#   1. 前进搜索, YOLO 分割模型检测裂缝
#   2. 记录最佳裂缝位置 (时间戳补偿 + odom)
#   3. 计算裂缝物理尺寸 (长度/宽度/距离)
#   4. 停车 -> 两段式返回到裂缝位置
#
# 前提: 已启动
#   ros2 launch ros_robot_controller ros_robot_controller.launch.py
#   ros2 launch controller controller.launch.py
#   realsense 相机运行中
#
# 用法:
#   python3 -u crack_detect.py --distance 1.5 --speed 0.2
#   python3 -u crack_detect.py --model /home/ubuntu/ros2_ws/YOLO_26n_crack.pt
import sys
import os
import queue
from collections import deque
import time
import argparse
import threading

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, HistoryPolicy, ReliabilityPolicy,
                        qos_profile_sensor_data)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
except ImportError:
    print('未找到 ultralytics, 请安装: pip3 install ultralytics', flush=True)
    sys.exit(1)


class CrackDetectNode(Node):
    def __init__(self, model_path, rgb_topic, depth_topic,
                 conf, imgsz, focal_px,
                 skip_frames, history_sec):
        super().__init__('crack_detect')
        self.lock = threading.Lock()

        # odom
        self.x = 0.0
        self.got_odom = False
        self.odom_history = deque(maxlen=int(history_sec * 50))

        # YOLO
        self.model = YOLO(model_path)
        self.conf = conf
        self.imgsz = imgsz
        self.f_px = focal_px
        self.skip_frames = skip_frames
        self.frame_idx = 0

        # 检测结果
        self.best_x = None
        self.best_dist = None
        self.best_length_mm = 0.0
        self.best_width_mm = 0.0
        self.best_score = 0.0
        self.det_count = 0

        # 图像
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_depth = None
        self.latest_msg = None
        self.frame_lock = threading.Lock()

        # 量化线程
        self.task_q = queue.Queue(maxsize=2)
        self.stop_event = threading.Event()
        self.quant_thread = threading.Thread(target=self._quant_worker, daemon=True)
        self.quant_thread.start()

        # ROS
        pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', pub_qos)
        self.create_subscription(Odometry, '/odom_raw', self.on_odom, 10)

        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, rgb_topic, self.on_rgb, sub_qos)
        self.create_subscription(Image, depth_topic, self.on_depth, sub_qos)
        self.result_pub = self.create_publisher(Image, '/crack_result', 10)

        self.timer = self.create_timer(1.0 / 5.0, self.process_frame)

    # ===== odom =====
    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        with self.lock:
            self.x = x
            self.got_odom = True
            self.odom_history.append((t, x))

    def get_x_at_time(self, target_t):
        with self.lock:
            if not self.odom_history:
                return self.x
            best = min(self.odom_history, key=lambda p: abs(p[0] - target_t))
            return best[1]

    # ===== 图像 =====
    def on_rgb(self, msg):
        try:
            self.latest_msg = msg
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB: {e}')

    def on_depth(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth: {e}')

    # ===== 量化线程 =====
    def _quant_worker(self):
        while not self.stop_event.is_set():
            try:
                mask, dist_mm = self.task_q.get(timeout=0.2)
                m = self._calc_metrics(mask)

                if dist_mm > 0:
                    px_to_mm = float(dist_mm) / self.f_px
                    m['length_mm'] = m['length_px'] * px_to_mm
                    m['avg_width_mm'] = m['avg_width_px'] * px_to_mm
                    m['max_width_mm'] = m['max_width_px'] * px_to_mm
                    m['distance_m'] = dist_mm / 1000.0
                else:
                    m['length_mm'] = m['avg_width_mm'] = 0.0
                    m['max_width_mm'] = m['distance_m'] = 0.0

                with self.lock:
                    self._latest_metrics = m

                self.task_q.task_done()
            except queue.Empty:
                continue

    def _calc_metrics(self, mask):
        area = int(mask.sum()) if mask is not None else 0
        if area <= 0:
            return {'area_px': 0, 'length_px': 0, 'avg_width_px': 0.0, 'max_width_px': 0.0}

        dist_map = cv2.distanceTransform(mask, cv2.DIST_L2, 3)
        max_w = 2.0 * float(dist_map.max())

        length = 0
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            length += cv2.arcLength(cnt, True) / 2.0

        avg_w = float(area) / float(length + 1e-6)
        return {'area_px': area, 'length_px': int(length),
                'avg_width_px': avg_w, 'max_width_px': max_w}

    # ===== 主处理循环 =====
    def process_frame(self):
        if self.latest_frame is None:
            return

        self.frame_idx += 1
        if self.skip_frames > 0 and self.frame_idx % (self.skip_frames + 1) != 0:
            return

        with self.frame_lock:
            frame = self.latest_frame.copy()
            depth = self.latest_depth.copy() if self.latest_depth is not None else None
            msg = self.latest_msg

        image_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        try:
            results = self.model.predict(source=frame, conf=self.conf,
                                         imgsz=self.imgsz, verbose=False)
        except Exception as e:
            self.get_logger().error(f'YOLO: {e}')
            return

        h, w = frame.shape[:2]
        best_det = None
        best_mask = None

        for r in results:
            if r.masks is not None and len(r.masks) > 0:
                for i, m in enumerate(r.masks.data):
                    cls = int(r.boxes.cls[i])
                    score = float(r.boxes.conf[i])
                    if score < self.conf:
                        continue

                    m_resized = cv2.resize(m.cpu().numpy(), (w, h),
                                           interpolation=cv2.INTER_NEAREST)
                    binary = (m_resized > 0.5).astype(np.uint8)

                    # 深度距离
                    dist_m = 0.0
                    if depth is not None:
                        depths = depth[binary > 0]
                        valid = depths[(depths > 100) & (depths < 10000)]
                        if valid.size > 0:
                            dist_m = float(np.median(valid)) / 1000.0

                    # bbox 面积
                    box = r.boxes.xyxy[i].cpu().numpy()
                    area = (box[2] - box[0]) * (box[3] - box[1])

                    if best_det is None or dist_m < best_det['dist']:
                        best_det = {'dist': dist_m, 'area': area,
                                    'score': score, 'cls': cls}
                        best_mask = binary

            elif r.boxes is not None and len(r.boxes) > 0:
                for i in range(len(r.boxes)):
                    cls = int(r.boxes.cls[i])
                    score = float(r.boxes.conf[i])
                    if score < self.conf:
                        continue
                    box = r.boxes.xyxy[i].cpu().numpy()
                    area = (box[2] - box[0]) * (box[3] - box[1])
                    if best_det is None or area > best_det['area']:
                        best_det = {'dist': 0.0, 'area': area,
                                    'score': score, 'cls': cls}

        # 更新最佳检测
        if best_det is not None:
            pose_x = self.get_x_at_time(image_t)
            dist = best_det['dist'] if best_det['dist'] > 0 else None

            if not self.task_q.full() and best_mask is not None:
                self.task_q.put_nowait((best_mask, best_det['dist'] * 1000))

            with self.lock:
                self.det_count += 1
                if dist is not None:
                    if self.best_dist is None or dist < self.best_dist:
                        self.best_x = pose_x
                        self.best_dist = dist
                        self.best_score = best_det['score']
                elif self.best_x is None:
                    self.best_x = pose_x
                    self.best_area = best_det['area']
                    self.best_score = best_det['score']

        # 发布标注图像
        annotated = results[0].plot()
        metrics = getattr(self, '_latest_metrics',
                          {'distance_m': 0, 'length_mm': 0,
                           'avg_width_mm': 0, 'max_width_mm': 0})
        self._draw_ui(annotated, metrics)
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = msg.header
        self.result_pub.publish(out_msg)

    def _draw_ui(self, img, m):
        f = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, f"Dist: {m.get('distance_m', 0):.2f} m",
                    (15, 35), f, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, f"Len: {m.get('length_mm', 0):.1f} mm",
                    (15, 65), f, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(img, f"AvgW: {m.get('avg_width_mm', 0):.2f} mm",
                    (15, 95), f, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(img, f"MaxW: {m.get('max_width_mm', 0):.2f} mm",
                    (15, 125), f, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

    # ===== 运动控制 =====
    def send_vel(self, v):
        msg = Twist()
        msg.linear.x = v
        self.cmd_pub.publish(msg)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main():
    parser = argparse.ArgumentParser(description='裂缝检测 + 定位返回')
    parser.add_argument('--model', type=str,
                        default='/home/ubuntu/car_control/YOLO_26n_crack.pt',
                        help='YOLO 权重路径')
    parser.add_argument('--rgb-topic', default='/camera/camera/color/image_raw')
    parser.add_argument('--depth-topic',
                        default='/camera/camera/aligned_depth_to_color/image_raw')
    parser.add_argument('--conf', type=float, default=0.25, help='置信度阈值')
    parser.add_argument('--imgsz', type=int, default=320, help='推理分辨率')
    parser.add_argument('--focal-px', type=float, default=500.0, help='相机焦距(px)')
    parser.add_argument('--distance', type=float, default=1.0,
                        help='最大前进距离(米)')
    parser.add_argument('--speed', type=float, default=0.2, help='前进速度(m/s)')
    parser.add_argument('--slow-speed', type=float, default=0.05,
                        help='慢速倒车速度(m/s)')
    parser.add_argument('--slow-zone', type=float, default=0.15,
                        help='慢速区距离(米)')
    parser.add_argument('--tolerance', type=float, default=0.02,
                        help='到位精度(米)')
    parser.add_argument('--no-return', action='store_true', help='不返回')
    parser.add_argument('--skip-frames', type=int, default=0,
                        help='跳帧(0=每帧)')
    parser.add_argument('--history-sec', type=float, default=5.0)
    args = parser.parse_args()

    if not os.path.exists(args.model):
        print(f'模型不存在: {args.model}', flush=True)
        sys.exit(1)

    rclpy.init()
    node = CrackDetectNode(
        model_path=args.model,
        rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic,
        conf=args.conf, imgsz=args.imgsz, focal_px=args.focal_px,
        skip_frames=args.skip_frames, history_sec=args.history_sec)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    print('等待 odom...', flush=True)
    for _ in range(100):
        if node.got_odom:
            break
        time.sleep(0.1)
    if not node.got_odom:
        print('未收到 odom', flush=True)
        rclpy.shutdown()
        return

    with node.lock:
        x_start = node.x
    print(f'初始 x = {x_start:.3f} m', flush=True)
    print(f'模型: {args.model}', flush=True)

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
            d = abs(x - x_start)

            if time.time() - last_print >= 0.3:
                bd = f'{best_dist:.2f}' if best_dist else '?'
                print(f'{time.time()-start:6.2f} | {x:+8.3f} | {d:8.3f} | '
                      f'{bd:>8} | {det_count:4d}', flush=True)
                last_print = time.time()

            if d >= args.distance:
                print(f'\n到达最大距离 {args.distance:.2f} m', flush=True)
                break
            time.sleep(0.05)

        node.stop()
        time.sleep(0.5)

        # ===== 结果 =====
        with node.lock:
            best_x = node.best_x
            best_dist = node.best_dist
            best_score = node.best_score
            det_count = node.det_count
            x_end = node.x

        print(f'\n搜索结束, 共 {det_count} 帧检测', flush=True)
        print(f'最终 x = {x_end:+.3f} m', flush=True)

        if best_x is None:
            print('未检测到裂缝', flush=True)
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
            dist_to = abs(x_end - best_x)
            print(f'\n找到裂缝:', flush=True)
            print(f'  位置: x = {best_x:+.3f} m', flush=True)
            print(f'  深度距离: {round(best_dist, 2) if best_dist else "?"} m', flush=True)
            print(f'  置信度: {best_score:.2f}', flush=True)
            print(f'  当前距裂缝: {dist_to:.3f} m', flush=True)

            if args.no_return:
                print('已停在当前位置', flush=True)
            else:
                print(f'\n返回裂缝位置...', flush=True)

                while True:
                    with node.lock:
                        x = node.x
                    remaining = abs(x - best_x)
                    if remaining <= args.slow_zone:
                        break
                    node.send_vel(-args.speed)
                    time.sleep(0.05)

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
                print(f'  已到达裂缝 x={x:.3f} m (误差 {abs(x-best_x):.3f} m)',
                      flush=True)

    except KeyboardInterrupt:
        print('\n手动停止', flush=True)
    finally:
        node.stop()
        node.stop_event.set()
        print('已停止', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
