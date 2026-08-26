import threading
import cv2
import numpy as np
import os
import time
import queue
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO


class YoloSegNode(Node):
    def __init__(self):
        super().__init__('yolo_seg_node')

        # --- 1. 参数声明 ---
        self.declare_parameter('input_topic', '/ascamera/camera_publisher/rgb0/image')
        self.declare_parameter('depth_topic', '/ascamera/camera_publisher/depth0/image_raw')
        self.declare_parameter('output_topic', '/yolo_result')

        # 相机焦距 (Pixel)，请根据你的相机内参修改。如果不确定，320-640 是常用范围
        self.declare_parameter('focal_length_px', 500.0)

        try:
            package_share_dir = get_package_share_directory('yolo_seg_ros2')
            default_model_path = os.path.join(package_share_dir, 'YOLO_26n_crack.pt')
        except Exception:
            default_model_path = 'YOLO_26n_crack.pt'

        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('process_fps', 5.0)
        self.declare_parameter('smooth_win', 5)

        # 获取参数
        self.f_px = self.get_parameter('focal_length_px').value
        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.imgsz = self.get_parameter('imgsz').value

        # --- 2. 初始化 ---
        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)

        # 量化数据：增加物理尺寸字段
        self.latest_metrics = {
            "area_px": 0, "length_px": 0, "avg_width_px": 0.0, "max_width_px": 0.0,
            "length_mm": 0.0, "avg_width_mm": 0.0, "max_width_mm": 0.0, "distance_m": 0.0
        }
        self.hist_metrics = {k: deque(maxlen=self.get_parameter('smooth_win').value) for k in
                             self.latest_metrics.keys()}

        self.task_q = queue.Queue(maxsize=2)
        self.stop_event = threading.Event()
        self.quant_thread = threading.Thread(target=self.quant_worker, daemon=True)
        self.quant_thread.start()

        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_depth_frame = None
        self.latest_msg = None
        self.processing = False

        # --- 3. ROS 订阅发布 ---
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, self.get_parameter('input_topic').value, self.image_callback, sub_qos)
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self.depth_callback, sub_qos)
        self.publisher = self.create_publisher(Image, self.get_parameter('output_topic').value, 10)

        timer_period = 1.0 / max(self.get_parameter('process_fps').value, 0.1)
        self.timer = self.create_timer(timer_period, self.process_latest_frame)

        self.get_logger().info('YOLO Real-World Measurement Node Started.')

    # ===========================
    # 核心量化工作线程
    # ===========================
    def quant_worker(self):
        while not self.stop_event.is_set():
            try:
                # 获取任务：mask 和 对应的平均深度值
                mask01, dist_mm = self.task_q.get(timeout=0.2)

                # 1. 计算像素指标
                m = self.calculate_pixel_metrics(mask01)

                # 2. 物理换算 (基于当前深度)
                # 换算比例系数: mm/px = 距离 / 焦距
                if dist_mm > 0:
                    px_to_mm = float(dist_mm) / self.f_px
                    m["length_mm"] = m["length_px"] * px_to_mm
                    m["avg_width_mm"] = m["avg_width_px"] * px_to_mm
                    m["max_width_mm"] = m["max_width_px"] * px_to_mm
                    m["distance_m"] = dist_mm / 1000.0
                else:
                    m["length_mm"] = m["avg_width_mm"] = m["max_width_mm"] = m["distance_m"] = 0.0

                # 3. 更新平滑队列
                with self.frame_lock:
                    for k, v in m.items():
                        self.hist_metrics[k].append(v)
                        self.latest_metrics[k] = float(np.mean(self.hist_metrics[k]))

                self.task_q.task_done()
            except queue.Empty:
                continue

    def calculate_pixel_metrics(self, mask01):
        area_px = int(mask01.sum())
        if area_px <= 0:
            return {"area_px": 0, "length_px": 0, "avg_width_px": 0.0, "max_width_px": 0.0}

        dist_map = cv2.distanceTransform(mask01, cv2.DIST_L2, 3)
        max_w_px = 2.0 * float(dist_map.max())

        length_px = 0
        if hasattr(cv2, 'ximgproc'):
            skeleton = cv2.ximgproc.thinning((mask01 * 255).astype(np.uint8))
            length_px = int((skeleton > 0).sum())
        else:
            contours, _ = cv2.findContours(mask01, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours: length_px += cv2.arcLength(cnt, True) / 2.0

        avg_w_px = float(area_px) / float(length_px + 1e-6)
        return {"area_px": area_px, "length_px": int(length_px), "avg_width_px": avg_w_px, "max_width_px": max_w_px}

    # ===========================
    # 图像回调
    # ===========================
    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.latest_frame, self.latest_msg = cv_img, msg
        except Exception as e:
            self.get_logger().error(f'RGB Err: {e}')

    def depth_callback(self, msg):
        try:
            self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth Err: {e}')

    def process_latest_frame(self):
        if self.processing or self.latest_frame is None: return

        with self.frame_lock:
            frame, depth = self.latest_frame.copy(), (
                self.latest_depth_frame.copy() if self.latest_depth_frame is not None else None)
            msg, m_display = self.latest_msg, self.latest_metrics.copy()
            self.latest_frame = None

        self.processing = True
        try:
            results = self.model.predict(source=frame, conf=self.conf_threshold, imgsz=self.imgsz, verbose=False)
            h, w = frame.shape[:2]

            # --- 提取 Mask 并计算该区域的平均距离 ---
            if results[0].masks is not None and len(results[0].masks) > 0 and depth is not None:
                combined_mask = np.zeros((h, w), dtype=np.uint8)
                masks_data = results[0].masks.data.cpu().numpy()
                for m in masks_data:
                    m_resized = cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)
                    combined_mask = np.maximum(combined_mask, (m_resized > 0.5).astype(np.uint8))

                # 获取 Mask 区域内的深度中值（mm）
                mask_depths = depth[combined_mask > 0]
                valid_depths = mask_depths[(mask_depths > 100) & (mask_depths < 10000)]
                avg_dist = np.median(valid_depths) if valid_depths.size > 0 else 0.0

                if not self.task_q.full(): self.task_q.put_nowait((combined_mask, avg_dist))
            else:
                if not self.task_q.full(): self.task_q.put_nowait((np.zeros((h, w), dtype=np.uint8), 0.0))

            # --- 绘制 UI (移除 FPS) ---
            annotated_frame = results[0].plot()
            self.draw_measurement_board(annotated_frame, m_display)

            out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            out_msg.header = msg.header
            self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Loop Err: {e}')
        finally:
            self.processing = False

    def draw_measurement_board(self, img, m):
        """简洁版显示看板：无背景，无底部提示行"""
        color = (0, 255, 0)  # 绿色
        white = (255, 255, 255)  # 白色
        f = cv2.FONT_HERSHEY_SIMPLEX

        # 1. 距离显示 (白字)
        cv2.putText(img, f"Distance: {m['distance_m']:.2f} m", (25, 45), f, 0.7, white, 2, cv2.LINE_AA)

        # 绘制一条装饰性的分割线（可选，如果想全屏最简可以删掉下面这行）
        cv2.line(img, (25, 55), (280, 55), (150, 150, 150), 1)

        # 2. 物理尺寸显示 (绿字)
        # 增加了粗细和抗锯齿，确保在无背景时也清晰
        cv2.putText(img, f"Length: {m['length_mm']:.1f} mm", (25, 85), f, 0.6, color, 2, cv2.LINE_AA)
        cv2.putText(img, f"Avg Width: {m['avg_width_mm']:.2f} mm", (25, 115), f, 0.6, color, 2, cv2.LINE_AA)
        cv2.putText(img, f"Max Width: {m['max_width_mm']:.2f} mm", (25, 145), f, 0.6, color, 2, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_event.set()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()