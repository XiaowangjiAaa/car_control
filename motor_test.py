#!/usr/bin/env python3
# encoding: utf-8
# 电机速度测试: 按预设速度表依次前进/后退
#
# 用法:
#   python3 motor_test.py                          默认速度表
#   python3 motor_test.py --fwd 0.5:1,1:1,2:2     自定义前进
#   python3 motor_test.py --rev 0.5:1,1:1          自定义后退
#   python3 motor_test.py --no-rev                 只跑前进
import sys
import time
import argparse

sys.path.insert(0, '/home/ubuntu/shared')
sys.path.insert(0, '/home/ubuntu/.local/lib/python3.10/site-packages')
import ros_robot_controller_sdk as rrc

# 实测结果: M2- = 前进, M4+ = 前进
STOP_ALL = [[1, 0.0], [2, 0.0], [3, 0.0], [4, 0.0]]


def set_motors(board, m2, m4):
    board.set_motor_speed([
        [1, 0.0],
        [2, m2],
        [3, 0.0],
        [4, m4],
    ])


def run_seq(board, speed_table, direction):
    """direction: 'fwd' 或 'rev'"""
    sign = 1 if direction == 'fwd' else -1
    label = '前进' if direction == 'fwd' else '后退'

    print(f'\n{"="*40}')
    print(f'  {label}')
    print(f'{"="*40}')

    for i, (speed, duration) in enumerate(speed_table):
        m2 = -speed * sign   # M2负号=前进
        m4 = speed * sign    # M4正号=前进

        print(f'\n[{i+1}/{len(speed_table)}] {label} {speed:.2f} rps {duration}s')
        print(f'  M2={m2:+.2f}  M4={m4:+.2f}')

        set_motors(board, m2, m4)
        time.sleep(duration)

        board.set_motor_speed(STOP_ALL)
        print(f'  停')
        time.sleep(0.5)


def parse_table(s):
    result = []
    for item in s.split(','):
        parts = item.strip().split(':')
        speed = float(parts[0])
        duration = float(parts[1]) if len(parts) > 1 else 1.0
        result.append((speed, duration))
    return result


def main():
    parser = argparse.ArgumentParser(description='电机速度测试')
    parser.add_argument('--fwd', type=str, default=None,
                        help='前进速度表,如 0.5:1,1:1,2:2')
    parser.add_argument('--rev', type=str, default=None,
                        help='后退速度表,如 0.5:1,1:1')
    parser.add_argument('--no-rev', action='store_true',
                        help='只跑前进')
    args = parser.parse_args()

    board = rrc.Board()
    print('串口已打开')

    fwd = parse_table(args.fwd) if args.fwd else [(0.5, 1), (1.0, 1), (2.0, 2)]
    rev = parse_table(args.rev) if args.rev else [(0.5, 1), (1.0, 1), (2.0, 2)]

    try:
        run_seq(board, fwd, 'fwd')

        if not args.no_rev:
            input('\n按 Enter 开始后退...')
            run_seq(board, rev, 'rev')

        print(f'\n{"="*40}')
        print('测试完成')
        print(f'{"="*40}')
    finally:
        board.set_motor_speed(STOP_ALL)
        print('已停止')


if __name__ == '__main__':
    main()
