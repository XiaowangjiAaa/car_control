#!/bin/bash
# 启动 bringup + 键盘控制
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/realsense_ws/install/setup.bash 2>/dev/null

echo "=== 启动 bringup (ros_robot_controller + odom_publisher + realsense) ==="
ros2 launch bringup bringup.launch.py &
BRINGUP_PID=$!
sleep 6

echo "=== 启动键盘控制 ==="
python3 ~/car_control/teleop.py

echo "=== 停止 bringup ==="
kill $BRINGUP_PID 2>/dev/null
wait $BRINGUP_PID 2>/dev/null
