#!/bin/bash

set -e

echo "========================================"
echo " TurtleBot3 Nav2 项目启动"
echo "========================================"

# ROS 2 环境
source /opt/ros/humble/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source $PROJECT_DIR/install/setup.bash

# TurtleBot3 型号
export TURTLEBOT3_MODEL=burger

# 地图
MAP_FILE="$PROJECT_DIR/maps/turtlebot3_room.yaml"

# Nav2 参数
PARAM_FILE="$PROJECT_DIR/config/burger_nav2.yaml"

# ---------- [0/4] 清理残留进程 ----------
# 重复启动时残留的 Gazebo/Nav2 会导致 TF 冲突、端口占用、
# AMCL 定位漂移等问题。启动前先清理，保证幂等可重启。
echo "[0/4] 清理残留进程..."
pkill -9 -f "gzserver|gzclient" 2>/dev/null || true
pkill -9 -f "ros2 launch" 2>/dev/null || true
pkill -9 -f "controller_server|planner_server|bt_navigator|amcl|lifecycle_manager" 2>/dev/null || true
pkill -9 -f "rviz|robot_state_publisher|multi_point_nav" 2>/dev/null || true
sleep 2

# 在新终端窗口运行命令；无 GUI 会话时回退到后台运行
run_in_terminal() {
    if [ -n "$DISPLAY" ] && command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal -- bash -c "$1" || return 1
    else
        echo "  （无图形会话或 gnome-terminal 不可用，改为后台运行）"
        nohup bash -c "$1" > "$2" 2>&1 &
        return 0
    fi
}

echo "[1/4] 启动 Gazebo..."

run_in_terminal "
source /opt/ros/humble/setup.bash
source $PROJECT_DIR/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
" /tmp/gazebo_launch.log

echo "等待 Gazebo 就绪（最长 60 秒）..."

# 自适应等待：/scan 话题出现即表示 Gazebo 仿真就绪，
# 替代固定 sleep 8（慢机器上 8 秒可能不够，快机器上浪费等待）
SCAN_OK=0
for i in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -qx "/scan"; then
        SCAN_OK=1
        break
    fi
    sleep 2
done

if [ "$SCAN_OK" -ne 1 ]; then
    echo "错误：等待 60 秒后 /scan 仍未出现，Gazebo 启动失败" >&2
    echo "可查看 /tmp/gazebo_launch.log 排查" >&2
    exit 1
fi

echo "Gazebo 已就绪（/scan 可用）"
sleep 3

echo "[2/4] 启动 Nav2 + RViz..."

run_in_terminal "
source /opt/ros/humble/setup.bash
source $PROJECT_DIR/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  map:=$MAP_FILE \
  params_file:=$PARAM_FILE \
  use_sim_time:=True
" /tmp/nav2_launch.log

echo ""
echo "等待 Nav2 启动（最长 60 秒）..."

# 等待 lifecycle_manager_navigation 节点出现（带超时，
# 避免 Nav2 启动失败时脚本永久挂起）
NAV2_OK=0
for i in $(seq 1 30); do
    if ros2 node list 2>/dev/null | grep -q "lifecycle_manager_navigation"; then
        NAV2_OK=1
        break
    fi
    sleep 2
done

if [ "$NAV2_OK" -ne 1 ]; then
    echo "错误：等待 60 秒后 lifecycle_manager_navigation 未出现，Nav2 启动失败" >&2
    echo "可查看 /tmp/nav2_launch.log 排查" >&2
    exit 1
fi

echo "Nav2 lifecycle_manager 已启动，提前发布 AMCL 初始位姿..."

# 在 lifecycle_manager 激活 planner_server（需要 map→odom TF）之前发布初始位姿
# 否则 planner_server 的 global_costmap 会因等待 map frame 而卡住，
# 导致 bt_navigator 无法激活（AMCL/Nav2 启动死锁）
# --once 会等待订阅者出现，-w 30 限制最长等待 30 秒防止挂起
timeout 30 ros2 topic pub --once -w 1 /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
  || echo "警告：初始位姿发布超时，依赖 AMCL set_initial_pose 参数兜底"

echo "已发布 AMCL 初始位姿，等待 Nav2 Active..."

# 等待 lifecycle_manager 激活所有它管理的节点（最长 90 秒）
# 注意：velocity_smoother 不在 lifecycle_manager 管理列表中，需要手动激活
BT_ACTIVE=0
for i in $(seq 1 45); do
    STATE=$(ros2 lifecycle get /bt_navigator 2>/dev/null || true)

    if echo "$STATE" | grep -q "active"; then
        BT_ACTIVE=1
        break
    fi

    # 如果 bt_navigator 卡在 inactive（velocity_smoother 未激活导致 controller 失败），
    # 主动激活 velocity_smoother（多次尝试，因为它可能尚未 configure）
    if echo "$STATE" | grep -q "inactive"; then
        echo "bt_navigator 处于 inactive，尝试激活 velocity_smoother..."
        ros2 lifecycle set /velocity_smoother configure 2>/dev/null || true
        ros2 lifecycle set /velocity_smoother activate 2>/dev/null || true
        sleep 1
        # 激活 velocity_smoother 后 bt_navigator 可能变 inactive，重激活
        ros2 lifecycle set /bt_navigator activate 2>/dev/null || true
        sleep 2
        continue
    fi

    sleep 2
done

if [ "$BT_ACTIVE" -ne 1 ]; then
    echo "错误：等待 90 秒后 bt_navigator 仍未进入 Active 状态" >&2
    echo "可查看 /tmp/nav2_launch.log 排查" >&2
    exit 1
fi

echo ""
echo "========================================"
echo " Nav2 已进入 Active 状态"
echo "========================================"

# 确保 velocity_smoother 激活（它不在 lifecycle_manager 管理列表中）
VS_STATE=$(ros2 lifecycle get /velocity_smoother 2>/dev/null || echo "unknown")
if ! echo "$VS_STATE" | grep -q "active"; then
    echo "激活 velocity_smoother（非 lifecycle_manager 管理）..."
    ros2 lifecycle set /velocity_smoother configure 2>/dev/null || true
    ros2 lifecycle set /velocity_smoother activate 2>/dev/null || true
    sleep 1
    # 激活 velocity_smoother 可能导致 bt_navigator 变 inactive，重激活
    ros2 lifecycle set /bt_navigator activate 2>/dev/null || true
    sleep 2
fi

# 重发 AMCL 初始位姿（此时 AMCL 已 active，确保收到）
# config 中已设 set_initial_pose:true，此处为额外保障
echo "重发 AMCL 初始位姿（AMCL 已 active）..."
timeout 15 ros2 topic pub --once -w 1 /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
  || echo "警告：初始位姿重发超时，首次发布已由 AMCL set_initial_pose 覆盖"
sleep 2

echo ""
echo "[3/4] 等待 AMCL 定位确认（最长 30 秒）..."

# AMCL 在机器人移动前不发布 /amcl_pose；tf2_echo 也不退出。
# 用 tf2_ros tf2_echo + timeout 检查 map→odom 是否可用
# （grep /tf 不可靠：AMCL 发布 map→odom 频率低，单次 echo 可能只有 odom→base）
AMCL_OK=0
for i in $(seq 1 10); do
    if timeout 3 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q "Translation"; then
        AMCL_OK=1
        break
    fi
    sleep 1
done

if [ "$AMCL_OK" -ne 1 ]; then
    echo "错误：等待 30 秒后 map→odom TF 仍未建立，AMCL 定位失败" >&2
    exit 1
fi

echo "AMCL 定位成功（map→odom TF 已建立）"

echo ""
echo "========================================"
echo " AMCL 初始化完成"
echo "========================================"

echo ""
echo "[4/4] 自动启动多点导航..."
echo ""
echo "路线：P1 → P2 → P3 → P4"
echo ""

python3 $PROJECT_DIR/scripts/multi_point_nav.py

echo ""
echo "========================================"
echo " TurtleBot3 多点导航任务结束"
echo "========================================"
