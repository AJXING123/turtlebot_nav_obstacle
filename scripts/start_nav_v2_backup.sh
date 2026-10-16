#!/bin/bash

set -e

echo "========================================"
echo " TurtleBot3 Nav2 项目启动"
echo "========================================"

# ROS 2 环境
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

# TurtleBot3 型号
export TURTLEBOT3_MODEL=burger

# 地图
MAP_FILE="$HOME/turtlebot3_ws/maps/turtlebot3_room.yaml"

# Nav2 参数
PARAM_FILE="$HOME/turtlebot3_ws/config/burger_nav2.yaml"

echo "[1/3] 启动 Gazebo..."

gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
"

echo "等待 Gazebo 启动..."
sleep 8

echo "[2/3] 启动 Nav2 + RViz..."

gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  map:=$MAP_FILE \
  params_file:=$PARAM_FILE \
  use_sim_time:=True
"

echo ""
echo "等待 Nav2 启动..."

# 等待 bt_navigator 节点出现
until ros2 node list 2>/dev/null | grep -q "^/bt_navigator$"; do
    sleep 2
done

echo "bt_navigator 已启动，等待 Nav2 Active..."

# 等待 bt_navigator 进入 Active
while true; do
    STATE=$(ros2 lifecycle get /bt_navigator 2>/dev/null || true)

    if echo "$STATE" | grep -q "active"; then
        break
    fi

    sleep 2
done

echo ""
echo "========================================"
echo " Nav2 已进入 Active 状态"
echo "========================================"
echo ""
echo "请在 RViz 中使用："
echo "  2D Pose Estimate"
echo ""
echo "设置机器人初始位姿。"
echo "设置完成后，再执行："
echo ""
echo "python3 ~/turtlebot3_ws/scripts/multi_point_nav.py"
echo ""
echo "========================================"
