#!/usr/bin/env python3
# encoding: utf-8
# 默认测试: 直行3s@1.5 -> 左打30°舵角转3s@1 -> 归位: 倒车转3s@1 -> 后退3s@1.5
# 用法(容器内, 需已启动 realsense, 停止 ros_robot_controller 服务):
#   python3 -u round_trip.py
#   python3 -u round_trip.py --forward1 3 --turn-angle 30 --turn-time 3 --turn-dir left --turn-speed 1 --speed 1.5
#   python3 -u round_trip.py --turn-time 0 --turn-angle 90     # yaw 闭环转 90°
import sys
import os
import time
import argparse
import threading

for _d in [
    '/home/ubuntu/shared',
    '/home/pi/hiwonder-toolbox',
    os.path.expanduser('~/hiwonder-toolbox'),
    '/home/xiaowangji/raspberrypi/hiwonder-toolbox',
]:
    if os.path.isdir(_d):
        sys.path.insert(0, _d)
        break
import ros_robot_controller_sdk as rrc

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
except ImportError:
    print('未找到 rclpy (ROS2)。本脚本需要 IMU 数据, 请在 MentorPi 容器内运行:', flush=True)
    print('  docker exec -it MentorPi /bin/zsh -c "source ~/.zshrc; cd ~/shared && python3 -u round_trip.py ..."', flush=True)
    sys.exit(1)

MOTOR_TYPE_ACKER = 0x02
STEER_SERVO_ID = 2
STEER_CENTER = 1500
# 实测: M2负号=前进, M4正号=前进
MOTOR_LEFT_REAR = 2
MOTOR_RIGHT_REAR = 4
PULSE_MIN = 1000
PULSE_MAX = 2000
DEG = 180.0 / 3.14159265358979

def clamp(pulse):
    return max(PULSE_MIN, min(PULSE_MAX, int(pulse)))

class ImuReader(Node):
    def __init__(self, topic):
        super().__init__('imu_reader')
        self.lock = threading.Lock()
        self.gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.ts = 0.0
        self.create_subscription(Imu, topic, self.on_imu, qos_profile_sensor_data)

    def on_imu(self, msg):
        with self.lock:
            self.gyro['x'] = msg.angular_velocity.x
            self.gyro['y'] = msg.angular_velocity.y
            self.gyro['z'] = msg.angular_velocity.z
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

def set_motors(board, speed, forward):
    if forward:
        left, right = -speed, speed   # M2负号=前进, M4正号=前进
    else:
        left, right = speed, -speed
    board.set_motor_speed([
        [1, 0],
        [MOTOR_LEFT_REAR, left],
        [3, 0],
        [MOTOR_RIGHT_REAR, right],
    ])

def stop_all(board):
    board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])

def straight_segment(board, est, duration, forward, speed, kp, steer_sign, target_heading, servo_id, step=0.02):
    start = time.time()
    corr_sign = steer_sign if forward else -steer_sign
    last_steer = 0.0
    last_print = 0.0
    print('{}直行 {} 秒 (目标航向 {:+.1f}°) ...'.format('前' if forward else '后', duration, target_heading * DEG), flush=True)
    while time.time() - start < duration:
        est.update()
        err = target_heading - est.yaw
        if time.time() - last_steer >= 0.1:
            pulse = clamp(STEER_CENTER + corr_sign * kp * err * DEG)
            board.pwm_servo_set_position(0.1, [[servo_id, pulse]])
            last_steer = time.time()
        set_motors(board, speed, forward)
        if time.time() - last_print >= 0.5:
            print('  [{}] t={:4.1f}s yaw={:+6.1f} head_err={:+5.1f}'.format(
                '前' if forward else '后', time.time() - start, est.yaw * DEG, err * DEG), flush=True)
            last_print = time.time()
        time.sleep(step)
    stop_all(board)
    print('  -> yaw = {:+.1f} deg'.format(est.yaw * DEG), flush=True)

def turn_segment(board, est, angle_deg, speed, forward, steer_pulse, base_yaw, undo, servo_id, timeout, step=0.02):
    if angle_deg < 1.0:
        return
    start = time.time()
    yaw0 = est.yaw
    last_steer = 0.0
    last_print = 0.0
    timed_out = False
    label = '前向转弯' if forward else '倒车转弯'
    if undo:
        label += '(撤销)'
    print('{} {}° ...'.format(label, angle_deg), flush=True)
    while True:
        est.update()
        y = est.yaw
        if time.time() - last_steer >= 0.1:
            board.pwm_servo_set_position(0.1, [[servo_id, steer_pulse]])
            last_steer = time.time()
        set_motors(board, speed, forward)
        if not undo:
            done = abs(y - base_yaw) * DEG >= angle_deg
        else:
            sign = 1.0 if (y - base_yaw) >= 0 else -1.0
            done = (y - base_yaw) * sign * DEG <= 1.0 and abs(y - base_yaw) * DEG >= 1.0
        timed_out = time.time() - start > timeout
        if done or timed_out:
            break
        if time.time() - last_print >= 0.5:
            print('  [{}] t={:4.1f}s yaw={:+6.1f} deg'.format(label, time.time() - start, y * DEG), flush=True)
            last_print = time.time()
        time.sleep(step)
    stop_all(board)
    if timed_out:
        print('警告: {} 超时未完成!'.format(label), flush=True)
    elapsed = time.time() - start
    rate = (est.yaw - yaw0) * DEG / elapsed if elapsed > 0 else 0.0
    print('  -> yaw = {:+.1f} deg, 平均角速度 {:+.1f} deg/s'.format(est.yaw * DEG, rate), flush=True)
    if not undo and abs(rate) < 3.0:
        print('提示: 角速度很低, 检查 1) 前轮是否真的随舵机转动(用 steer_test sweep) '
              '2) 加大 --turn-speed 3) 转弯半径是否过大', flush=True)

def turn_segment_timed(board, est, steer_angle_deg, duration, forward, speed, steer_pulse, servo_id, step=0.02):
    start = time.time()
    yaw0 = est.yaw
    last_steer = 0.0
    last_print = 0.0
    label = '前向转弯' if forward else '倒车转弯'
    print('{}: 舵角 {:+d}°, {} 秒 @ speed={} ...'.format(
        label, int(steer_angle_deg), duration, speed), flush=True)
    while time.time() - start < duration:
        est.update()
        if time.time() - last_steer >= 0.1:
            board.pwm_servo_set_position(0.1, [[servo_id, steer_pulse]])
            last_steer = time.time()
        set_motors(board, speed, forward)
        if time.time() - last_print >= 0.5:
            print('  [{}] t={:4.1f}s yaw={:+6.1f} deg'.format(
                label, time.time() - start, est.yaw * DEG), flush=True)
            last_print = time.time()
        time.sleep(step)
    stop_all(board)
    rate = (est.yaw - yaw0) * DEG / duration if duration > 0 else 0.0
    print('  -> yaw = {:+.1f} deg, 平均角速度 {:+.1f} deg/s'.format(est.yaw * DEG, rate), flush=True)

def main():
    parser = argparse.ArgumentParser(description='MentorPi_Acker 多段路径行驶 + 归位测试')
    parser.add_argument('--forward1', type=float, default=3.0, help='第1段直行时长(秒), 默认 3')
    parser.add_argument('--forward2', type=float, default=0.0, help='第2段直行时长(秒), 默认 0 (无)')
    parser.add_argument('--speed', type=float, default=1.5, help='直行速度, 默认 1.5')
    parser.add_argument('--turn-angle', type=float, default=30.0, help='转弯舵角(度), 默认 30')
    parser.add_argument('--turn-time', type=float, default=3.0,
                        help='转弯时长(秒), 默认 3; 设 0 则改为 yaw 闭环转到 --turn-angle 度停')
    parser.add_argument('--turn-dir', choices=['left', 'right'], default='left', help='转弯方向, 默认 left')
    parser.add_argument('--turn-sign', type=int, default=1, help='转弯方向符号 1/-1, 转向反了改 -1')
    parser.add_argument('--turn-speed', type=float, default=1.0, help='转弯速度, 默认 1.0')
    parser.add_argument('--kp', type=float, default=5.0, help='直行纠偏增益(脉冲/度), 默认 5')
    parser.add_argument('--steer-sign', type=int, default=1, help='直行纠偏符号 1/-1')
    parser.add_argument('--yaw-sign', type=int, default=1, help='yaw 符号 1/-1')
    parser.add_argument('--ppd', type=float, default=10.0, help='转弯脉冲/度, 默认 10')
    parser.add_argument('--servo-id', type=int, default=STEER_SERVO_ID, help='转向舵机通道, 默认 2')
    parser.add_argument('--calib', type=float, default=2.0, help='静止标定时长(秒), 默认 2')
    parser.add_argument('--turn-timeout', type=float, default=30.0, help='转弯超时(秒), 默认 30')
    parser.add_argument('--topic', default='/camera/camera/imu')
    args = parser.parse_args()

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
    board.pwm_servo_set_position(0.5, [[args.servo_id, STEER_CENTER]])
    time.sleep(0.5)

    try:
        print('\n=== 出发路径 ===', flush=True)
        straight_segment(board, est, args.forward1, True, args.speed, args.kp,
                         args.steer_sign, 0.0, args.servo_id)

        turn_sign = args.turn_sign if args.turn_dir == 'left' else -args.turn_sign
        steer_pulse = clamp(STEER_CENTER + turn_sign * args.turn_angle * args.ppd)
        base_yaw = est.yaw
        if args.turn_time > 0:
            turn_segment_timed(board, est, args.turn_angle, args.turn_time,
                               True, args.turn_speed, steer_pulse, args.servo_id)
        else:
            turn_segment(board, est, args.turn_angle, args.turn_speed, True, steer_pulse,
                         base_yaw, undo=False, servo_id=args.servo_id, timeout=args.turn_timeout)
        h2 = est.yaw
        time.sleep(0.3)

        if args.forward2 > 0:
            straight_segment(board, est, args.forward2, True, args.speed, args.kp,
                             args.steer_sign, h2, args.servo_id)

        print('\n=== 归位(原路返回) ===', flush=True)
        if args.forward2 > 0:
            straight_segment(board, est, args.forward2, False, args.speed, args.kp,
                             args.steer_sign, h2, args.servo_id)
            time.sleep(0.3)

        if args.turn_time > 0:
            turn_segment_timed(board, est, args.turn_angle, args.turn_time,
                               False, args.turn_speed, steer_pulse, args.servo_id)
        else:
            turn_segment(board, est, args.turn_angle, args.turn_speed, False, steer_pulse,
                         base_yaw, undo=True, servo_id=args.servo_id, timeout=args.turn_timeout)
        time.sleep(0.3)

        straight_segment(board, est, args.forward1, False, args.speed, args.kp,
                         args.steer_sign, base_yaw, args.servo_id)

        print('\n=== 归位完成 ===', flush=True)
        print('最终 yaw = {:+.1f} deg (理论 ~0), 请目测小车是否回到起点'.format(est.yaw * DEG), flush=True)
    except KeyboardInterrupt:
        print('\n手动停止', flush=True)
    finally:
        stop_all(board)
        board.pwm_servo_set_position(0.5, [[args.servo_id, STEER_CENTER]])
        time.sleep(0.3)
        board.port.close()
        imu.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
