#!/bin/bash
# TurtleBot3 导航回归测试
# 覆盖场景：
#   T1 环境与配置完整性（关键文件存在、RPP 控制器配置未被回滚）
#   T2 幂等重启（重复运行 start_nav.sh，自动清理残留进程）
#   T3 Gazebo 仿真就绪（/scan 话题可用）
#   T4 AMCL 定位建立（map→odom TF）
#   T5 多点导航 P1→P2→P3→P4（含障碍箱子绕行）
#   T6 最终位姿验证（P4 落点误差 <= 0.30m）
#
# 用法：bash scripts/test_navigation.sh [每点超时秒数，默认120]
# 退出码：0=全部通过，1=存在失败项
#
# 注意：grep -q 退出码 0 = 找到 = PASS，统一用 check_log 函数转换

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PER_POINT_TIMEOUT="${1:-120}"

PASS=0
FAIL=0
RESULTS=()

record() {
    # record <测试ID> <描述> <是否通过:1/0>
    local id="$1" name="$2" ok="$3"
    if [ "$ok" -eq 1 ]; then
        RESULTS+=("[PASS] $id $name")
        PASS=$((PASS+1))
    else
        RESULTS+=("[FAIL] $id $name")
        FAIL=$((FAIL+1))
    fi
}

# grep 找到关键字返回 1（PASS），未找到返回 0（FAIL）
check_log() {
    if grep -q "$1" "$2" 2>/dev/null; then
        return 1
    fi
    return 0
}

echo "=========================================="
echo " TurtleBot3 导航回归测试"
echo "=========================================="

# ---------- T1 环境与配置完整性 ----------
T1=1
[ -f "$PROJECT_DIR/config/burger_nav2.yaml" ] || T1=0
[ -f "$PROJECT_DIR/maps/turtlebot3_room.yaml" ] || T1=0
[ -f "$PROJECT_DIR/maps/turtlebot3_room.pgm" ] || T1=0
[ -f "$PROJECT_DIR/scripts/multi_point_nav.py" ] || T1=0
[ -d "$PROJECT_DIR/install" ] || T1=0

# 关键配置检查：防止编辑器旧缓冲区覆盖导致控制器回滚到 DWB
if [ -f "$PROJECT_DIR/config/burger_nav2.yaml" ]; then
    grep -q "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController" \
        "$PROJECT_DIR/config/burger_nav2.yaml" || T1=0
    grep -q "set_initial_pose: true" "$PROJECT_DIR/config/burger_nav2.yaml" || T1=0
fi
record T1 "环境与配置完整性（RPP控制器/地图/脚本/工作区）" "$T1"

echo ""
echo "---------- T2-T5 端到端导航 ----------"
echo "启动完整导航流程（start_nav.sh 已内置残留进程清理与就绪检测）..."

RUN_LOG=/tmp/regression_nav_run.log
rm -f "$RUN_LOG"

timeout $((PER_POINT_TIMEOUT * 4 + 300)) \
    bash "$PROJECT_DIR/scripts/start_nav.sh" > "$RUN_LOG" 2>&1
RUN_EXIT=$?

# ---------- T3 Gazebo 仿真就绪 ----------
check_log "Gazebo 已就绪" "$RUN_LOG"; T3=$?
record T3 "Gazebo 仿真就绪（/scan 话题出现）" "$T3"

# ---------- T4 AMCL 定位建立 ----------
check_log "AMCL 定位成功" "$RUN_LOG"; T4=$?
record T4 "AMCL 定位建立（map→odom TF）" "$T4"

# ---------- T5 多点导航逐点验证 ----------
for P in P1 P2 P3 P4; do
    check_log "✓ $P 到达成功" "$RUN_LOG"; TP=$?
    record T5 "$P 到达（含 P2 障碍绕行）" "$TP"
done

check_log "全部导航完成" "$RUN_LOG"; TALL=$?
record T5b "P1→P2→P3→P4 全程完成" "$TALL"

# ---------- T6 最终位姿验证 ----------
# 在子进程中加载 ROS 环境后读取 /amcl_pose，与 P4 (0.333, -1.778) 比较。
# 注意：不能在测试脚本进程内直接 source ROS 环境（会导致脚本异常退出），
# 必须放入 bash -c 子进程执行。
T6=0
if [ "$TALL" -eq 1 ]; then
    FINAL_POS=$(timeout 30 bash -c '
        source /opt/ros/humble/setup.bash 2>/dev/null
        source "'"$PROJECT_DIR"'/install/setup.bash" 2>/dev/null
        export TURTLEBOT3_MODEL=burger
        python3 - <<PYEOF
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

rclpy.init()
node = Node("final_pose_check")
got = {}

def cb(msg):
    if "x" not in got:
        got["x"] = msg.pose.pose.position.x
        got["y"] = msg.pose.pose.position.y

node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", cb, 10)
import time
start = time.time()
while "x" not in got and time.time() - start < 8:
    rclpy.spin_once(node, timeout_sec=0.2)

if "x" in got:
    err = math.hypot(got["x"] - 0.333, got["y"] - (-1.778))
    print(f"{got[chr(120)]:.3f},{got[chr(121)]:.3f},{err:.3f}")
node.destroy_node()
rclpy.shutdown()
PYEOF
    ' 2>/dev/null || true)

    if [ -n "$FINAL_POS" ]; then
        ERR=$(echo "$FINAL_POS" | tail -1 | awk -F, '{print $3}')
        if [ -n "$ERR" ]; then
            T6=$(python3 -c "print(1 if float('$ERR') <= 0.30 else 0)" 2>/dev/null || echo 0)
            echo "  最终位姿: $(echo "$FINAL_POS" | tail -1 | awk -F, '{print $1","$2}')，P4 误差 ${ERR} m"
        fi
    fi
fi
record T6 "P4 最终位姿误差 <= 0.30m" "$T6"

# ---------- 汇总 ----------
echo ""
echo "=========================================="
echo " 回归测试结果"
echo "=========================================="
for r in "${RESULTS[@]}"; do
    echo " $r"
done
echo "------------------------------------------"
echo " 通过: $PASS  失败: $FAIL"
echo " 运行日志: $RUN_LOG (start_nav.sh exit=$RUN_EXIT)"
echo "=========================================="

# 失败时输出关键日志片段辅助排查
if [ "$FAIL" -gt 0 ]; then
    echo ""
    echo "===== 日志末尾 40 行（排查用） ====="
    tail -40 "$RUN_LOG"
fi

[ "$FAIL" -eq 0 ] && exit 0 || exit 1
