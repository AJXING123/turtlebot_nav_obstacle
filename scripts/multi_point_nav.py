#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseStamped


class MultiPointNavigator(Node):

    # 每个目标点最多尝试次数（含首次）
    MAX_ATTEMPTS = 3

    def __init__(self):
        super().__init__('multi_point_navigator')

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        # 清空 costmap 服务客户端（重试前调用，清除陈旧障碍标记）
        # 全局 costmap 清空后只剩静态地图+膨胀，NavFn 可重新规划
        # Nav2 Humble 的服务名带 costmap 名称后缀，类型为 ClearEntireCostmap
        self._clear_global = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'
        )
        self._clear_local = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'
        )

        # 反馈日志节流
        self._last_feedback_time = None

        # P1 → P2 → P3 → P4
        self.points = [
            {
                'name': 'P1',
                'x': -2.074,
                'y': -0.418,
                'z': 0.156,
                'w': 0.988
            },
            {
                'name': 'P2',
                'x': -1.770,
                'y': 0.741,
                'z': 0.666,
                'w': 0.746
            },
            {
                'name': 'P3',
                'x': 0.548,
                'y': 0.431,
                'z': -0.688,
                'w': 0.725
            },
            {
                'name': 'P4',
                'x': 0.333,
                'y': -1.778,
                'z': -0.826,
                'w': 0.564
            }
        ]

    def create_goal(self, point):
        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = point['x']
        pose.pose.position.y = point['y']
        pose.pose.position.z = 0.0

        pose.pose.orientation.z = point['z']
        pose.pose.orientation.w = point['w']

        goal_msg.pose = pose

        return goal_msg

    def feedback_callback(self, feedback_msg):
        now = self.get_clock().now()

        # 每 2 秒输出一次剩余距离，便于观察是否在持续推进
        if (self._last_feedback_time is not None and
                (now - self._last_feedback_time).nanoseconds < 2e9):
            return

        self._last_feedback_time = now

        distance = feedback_msg.feedback.distance_remaining
        eta = feedback_msg.feedback.estimated_time_remaining
        eta_sec = eta.sec + eta.nanosec * 1e-9

        self.get_logger().info(
            f"  剩余距离: {distance:.2f} m, 预计剩余: {eta_sec:.1f} s"
        )

    def navigate_to_point(self, point, attempt):
        self.get_logger().info(
            f"正在前往 {point['name']} "
            f"({point['x']}, {point['y']}) "
            f"[第 {attempt}/{self.MAX_ATTEMPTS} 次尝试]"
        )

        self._action_client.wait_for_server()

        # 每次尝试重置反馈节流时间
        self._last_feedback_time = None

        goal_msg = self.create_goal(point)

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(
            self,
            send_goal_future
        )

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                f"{point['name']} 目标被 Nav2 拒绝"
            )
            return False

        self.get_logger().info(
            f"{point['name']} 目标已接受"
        )

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result()

        if result.status == 4:
            self.get_logger().info(
                f"✓ {point['name']} 到达成功"
            )
            return True

        self.get_logger().warning(
            f"✗ {point['name']} 导航失败，"
            f"status={result.status}"
        )

        return False

    def clear_costmaps(self):
        """清空 local/global costmap 的 obstacle_layer，移除陈旧障碍标记。

        全局 costmap 清空后只剩 static_layer + inflation_layer，
        NavFn 可在干净的静态地图上重新规划，排除激光误判/AMCL 漂移
        导致的虚假障碍。后续激光会重新填充 obstacle_layer。
        """
        cleared = False

        for name, client in [('local', self._clear_local),
                              ('global', self._clear_global)]:
            if not client.service_is_ready():
                self.get_logger().warning(
                    f'{name} costmap clear 服务未就绪，跳过'
                )
                continue

            req = ClearEntireCostmap.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=2.0
            )

            if future.result() is not None:
                self.get_logger().info(f'{name} costmap 已清空')
                cleared = True
            else:
                self.get_logger().warning(
                    f'{name} costmap 清空超时'
                )

        # 给 costmap 一个更新周期，让静态层重新发布
        rclpy.spin_once(self, timeout_sec=0.5)

        return cleared


def main(args=None):

    rclpy.init(args=args)

    navigator = MultiPointNavigator()

    navigator.get_logger().info(
        '========================================'
    )
    navigator.get_logger().info(
        'TurtleBot3 多点定点导航开始'
    )
    navigator.get_logger().info(
        '路线：P1 → P2 → P3 → P4'
    )
    navigator.get_logger().info(
        f'每个点失败自动重试，最多 '
        f'{MultiPointNavigator.MAX_ATTEMPTS} 次'
    )
    navigator.get_logger().info(
        '========================================'
    )

    failed = False

    for point in navigator.points:

        success = False

        for attempt in range(1, navigator.MAX_ATTEMPTS + 1):

            success = navigator.navigate_to_point(point, attempt)

            if success:
                break

            if attempt < navigator.MAX_ATTEMPTS:
                navigator.get_logger().info(
                    f'{point["name"]} 清空 costmap 后重试...'
                )
                navigator.clear_costmaps()
                rclpy.spin_once(navigator, timeout_sec=1.0)

        if not success:
            navigator.get_logger().error(
                f'{point["name"]} 重试 '
                f'{navigator.MAX_ATTEMPTS} 次后仍失败，导航任务中止'
            )
            failed = True
            break

    if not failed:
        navigator.get_logger().info(
            '========================================'
        )
        navigator.get_logger().info(
            '🎉 P1 → P2 → P3 → P4 全部导航完成！'
        )
        navigator.get_logger().info(
            '========================================'
        )

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
