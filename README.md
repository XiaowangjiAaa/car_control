# car_control

MentorPi Acker 底盘控制脚本集合

---

## 环境启动

机器人 SSH 连接后依次执行（开 4 个终端）：

### 终端 1 — 底盘驱动

```bash
source ~/.zshrc
ros2 launch ros_robot_controller ros_robot_controller.launch.py
```

### 终端 2 — 控制器 + Odom

```bash
source ~/.zshrc
ros2 launch controller controller.launch.py
```

### 终端 3 — 相机（RealSense）

```bash
source ~/.zshrc
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true rgb_camera.color_profile:=640x480x15
```

### 终端 4 — YOLO 检测（仅苹果检测需要）

```bash
source ~/.zshrc
ros2 launch yolo_distance yolo_with_distance.launch.py \
  use_distance:=True distance_method:=region \
  model:=/home/ubuntu/ros2_ws/yolo11n_ncnn_model \
  imgsz_height:=320 imgsz_width:=416 device:=cpu \
  input_image_topic:=/camera/camera/color/image_raw
```

> **裂缝检测不需要终端 4**，`crack_detect.py` 自带 YOLO 推理。

---

## 脚本说明

### crack_detect.py — 裂缝检测 + 定位返回

自带 YOLO 模型，不需要启动终端 4。

```bash
python3 -u crack_detect.py --distance 1.5 --speed 0.2
```

用 NCNN 模型：

```bash
python3 -u crack_detect.py \
  --model /home/ubuntu/car_control/YOLO_26n_crack_ncnn_model \
  --distance 1.5 --speed 0.2
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--model` | `YOLO_26n_crack.pt` | YOLO 权重路径（.pt 或 ncnn_model 文件夹） |
| `--distance` | 1.0 | 最大前进距离(米) |
| `--speed` | 0.2 | 前进速度(m/s) |
| `--slow-speed` | 0.05 | 慢速返回速度(m/s) |
| `--slow-zone` | 0.15 | 进入慢速区的距离(米) |
| `--tolerance` | 0.02 | 到位精度(米) |
| `--conf` | 0.25 | 置信度阈值 |
| `--imgsz` | 320 | 推理分辨率 |
| `--focal-px` | 500.0 | 相机焦距(px) |
| `--rgb-topic` | `/camera/camera/color/image_raw` | RGB 话题 |
| `--depth-topic` | `/camera/camera/aligned_depth_to_color/image_raw` | 深度话题 |
| `--no-return` | - | 检测到裂缝后不返回 |
| `--skip-frames` | 0 | 跳帧(0=每帧检测) |

输出: `/crack_result` — 标注后的图像

**工作流程：**
1. 前进搜索，最大距离由 `--distance` 控制
2. YOLO 检测裂缝，记录最佳位置（距离最近的）
3. 停车，两段式倒车返回（远处快、近处慢）

---

### apple_spot_v3.py — 苹果定位返回（时间戳补偿版）

需要启动终端 1-4。

```bash
python3 -u apple_spot_v3.py --distance 1.5 --speed 0.2
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--det-topic` | `/yolo/detections_with_dist` | 检测话题 |
| `--class-name` | `apple` | 目标类别 |
| `--min-score` | 0.5 | 最低置信度 |
| `--distance` | 1.0 | 最大前进距离(米) |
| `--speed` | 0.2 | 前进速度(m/s) |
| `--history-sec` | 5.0 | odom 历史保留时长(秒) |

---

### apple_spot.py — 苹果定位返回（基础版）

```bash
python3 -u apple_spot.py --distance 1.5 --speed 0.2
```

---

### 其他脚本

| 脚本 | 说明 |
|------|------|
| `motor_test.py` | 电机速度测试 |
| `odom_cmdvel_test.py` | cmd_vel + odom 验证 |
| `odom_verify.py` | odom 原始验证 |
| `drive_straight.py` | IMU 直线行驶 + 返回 |
| `round_trip.py` | 多段往返 |
| `teleop.py` | 手柄遥控 |

---

## 电机方向（已验证）

| 电机 | 方向 |
|------|------|
| M1 | 无响应（故障） |
| M2 | 负值 = 前进 |
| M3 | 无响应（故障） |
| M4 | 正值 = 前进 |

## 架构

```
cmd_vel → ros_robot_controller → RRCLite(PID 10ms) → 电机
                                    ↓
                              /controller/cmd_vel (闭环后速度)
                                    ↓
                            odom_publisher_node → /odom_raw
```
