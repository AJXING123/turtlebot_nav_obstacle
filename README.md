# TurtleBot3 Nav2 多点导航避障项目

基于 ROS 2 Humble + TurtleBot3 Burger + Nav2 + Gazebo Classic 的自主导航项目。机器人在含**未建图静态障碍物**的房间地图中，按 P1 → P2 → P3 → P4 顺序完成多点导航，全程无人工干预。

详细技术报告见 [项目技术报告](docs/TurtleBot3_Nav2_Project_Report.md)（另有 [HTML 版](docs/TurtleBot3_Nav2_Project_Report.html)，在 GitHub 上显示为源码，克隆后用浏览器打开可正常阅读）。

## 项目特点

- **一键启动**：`scripts/start_nav.sh` 自动完成 Gazebo → Nav2+RViz → AMCL 初始位姿 → lifecycle Active 确认 → 多点导航全流程，带就绪检测与超时保护
- **绕障导航**：障碍箱（中心 (-1.92, 0.15)，0.8m × 0.3m × 0.5m）不在静态地图中，靠激光雷达实时感知绕行
- **局部控制器**：采用 Regulated Pure Pursuit (RPP)，解决了 DWB 在贴障碍场景下的「零线速度 + 微幅角速度振荡」问题
- **失败重试**：每个目标点失败自动清空 costmap 重试（最多 3 次）

## 环境依赖

| 组件 | 版本 |
|------|------|
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| Gazebo Classic | 11 |
| Python | 3.10 |

Nav2 相关包（`nav2_bringup`、`nav2_regulated_pure_pursuit_controller` 等）通过 Humble 官方 deb 安装：

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## 目录结构

```
turtlebot3_ws_obstacle/
├── config/
│   └── burger_nav2.yaml              # Nav2 参数（RPP 控制器、costmap、BT 配置）
├── maps/
│   └── turtlebot3_room.yaml/.pgm     # 静态地图（0.05m 分辨率）
├── scripts/
│   ├── start_nav.sh                  # 一键启动脚本
│   ├── multi_point_nav.py            # 多点导航 Action Client
│   └── test_navigation.sh            # 回归测试脚本
├── docs/                             # 项目技术报告
└── src/                              # TurtleBot3 ROS 包（含自定义修改）
    ├── turtlebot3/                   # 机器人描述、导航包装
    ├── turtlebot3_msgs/              # 消息定义
    ├── turtlebot3_simulations/       # Gazebo 仿真
    │   └── turtlebot3_gazebo/
    │       ├── launch/turtlebot3_world.launch.py   # ★ 已修改：加载障碍世界
    │       └── worlds/turtlebot3_world_obstacle.world  # ★ 自定义障碍物世界
    └── DynamixelSDK/                 # 舵机 SDK
```

## 快速开始

```bash
# 1. 克隆仓库
git clone https://github.com/AJXING123/turtlebot_nav_obstacle.git
cd turtlebot_nav_obstacle

# 2. 构建工作空间
source /opt/ros/humble/setup.bash
colcon build

# 3. 一键启动（自动完成仿真 + 导航 + 多点巡航）
./scripts/start_nav.sh
```

启动后机器人将从 (-2.0, -0.5) 出发，依次前往：

| 目标点 | x (m) | y (m) | 说明 |
|--------|-------|-------|------|
| P1 | -2.074 | -0.418 | 起点附近，验证导航链路 |
| P2 | -1.770 | 0.741 | **必经障碍点**，从箱子东侧绕行 |
| P3 | 0.548 | 0.431 | 房间东北区域 |
| P4 | 0.333 | -1.778 | 房间南部终点 |

## 修改目标点

编辑 [scripts/multi_point_nav.py](scripts/multi_point_nav.py) 中的 `self.points` 列表即可。

## 常见问题

| 现象 | 原因 |
|------|------|
| `Package 'turtlebot3_gazebo' not found` | 未执行 `colcon build` 或未 source `install/setup.bash` |
| Gazebo 启动后无机器人 | 查看 `/tmp/gazebo_launch.log` |
| 导航不动 / bt_navigator 未激活 | 查看 `/tmp/nav2_launch.log`，通常是 AMCL 初始位姿未建立 |
