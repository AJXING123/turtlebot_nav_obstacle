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

echo "[1/4] 启动 Gazebo..."

gnome-terminal -- bash -c "
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
"

echo "等待 Gazebo 启动..."
sleep 8

echo "[2/4] 启动 Nav2 + RViz..."

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
echo "[3/4] 自动设置 AMCL 初始位姿..."

# Gazebo 初始位置对应地图坐标
ros2 topic pub --once /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"

echo "已发布 AMCL 初始位姿"
echo "等待 AMCL 定位..."

# 等待 AMCL 发布定位结果
until ros2 topic echo /amcl_pose --once >/dev/null 2>&1; do
    sleep 2
done

echo "AMCL 定位成功"

echo ""
echo "========================================"
echo " AMCL 初始化完成"
echo "========================================"

echo ""
echo "[4/4] 自动启动多点导航..."
echo ""
echo "路线：P1 → P2 → P3 → P4"
echo ""

python3 ~/turtlebot3_ws/scripts/multi_point_nav.py

echo ""
echo "========================================"
echo " TurtleBot3 多点导航任务结束"
echo "========================================"
