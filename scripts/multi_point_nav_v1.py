#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class MultiPointNavigator(Node):

    def __init__(self):
        super().__init__('multi_point_navigator')

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

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

    def navigate_to_point(self, point):
        self.get_logger().info(
            f"正在前往 {point['name']} "
            f"({point['x']}, {point['y']})"
        )

        self._action_client.wait_for_server()

        goal_msg = self.create_goal(point)

        send_goal_future = self._action_client.send_goal_async(goal_msg)

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

        self.get_logger().error(
            f"✗ {point['name']} 导航失败，"
            f"status={result.status}"
        )

        return False


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
        '========================================'
    )

    for point in navigator.points:

        success = navigator.navigate_to_point(point)

        if not success:
            navigator.get_logger().error(
                '导航任务中止'
            )
            break

    else:
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
