#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Path


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class RegulatedPurePursuit(Node):

    def __init__(self):
        super().__init__('regulated_pure_pursuit')

        # ===============================
        # 参数（Nav2 风格）
        # ===============================
        self.lookahead_dist = 0.8

        self.max_linear = 2.0
        self.max_angular = 2.0

        self.k_yaw = 2.5

        # 原地转向滞回
        self.rotate_enter = 0.7   # 进入原地转
        self.rotate_exit = 0.3    # 退出原地转

        # 调速因子（Regulation）
        self.min_speed_ratio = 0.2

        self.goal_tolerance = 0.2

        # ===============================
        # 状态
        # ===============================
        self.robot_pose = None
        self.global_path = None
        self.target_idx = 0

        self.rotate_in_place = False

        # ===============================
        # ROS IO
        # ===============================
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.create_subscription(Path, '/plan', self.path_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info('Regulated Pure Pursuit started')

    # ===============================
    # 回调
    # ===============================
    def pose_cb(self, msg: Pose):
        self.robot_pose = msg

    def path_cb(self, msg: Path):
        self.global_path = msg.poses
        self.target_idx = 0
        self.rotate_in_place = False
        self.get_logger().info(f'Received path with {len(self.global_path)} points')

    # ===============================
    # 主循环
    # ===============================
    def update(self):
        if self.robot_pose is None or not self.global_path:
            return

        target = self.select_lookahead()
        if target is None:
            return

        # --- 计算目标方向 ---
        dx = target.x - self.robot_pose.x
        dy = target.y - self.robot_pose.y
        target_yaw = math.atan2(dy, dx)

        yaw_error = self.normalize_angle(
            target_yaw - self.robot_pose.theta
        )

        # ===============================
        # 原地转向（带滞回）
        # ===============================
        if self.rotate_in_place:
            if abs(yaw_error) < self.rotate_exit:
                self.rotate_in_place = False
            else:
                self.publish_cmd(0.0, self.k_yaw * yaw_error)
                return
        else:
            if abs(yaw_error) > self.rotate_enter:
                self.rotate_in_place = True
                self.publish_cmd(0.0, self.k_yaw * yaw_error)
                return

        # ===============================
        # Regulated 前进控制
        # ===============================
        # 角速度
        w = clamp(
            self.k_yaw * yaw_error,
            -self.max_angular,
            self.max_angular
        )

        # 角度越大，线速度越小（Nav2 核心思想）
        speed_ratio = max(
            self.min_speed_ratio,
            1.0 - abs(w) / self.max_angular
        )

        v = self.max_linear * speed_ratio

        # 到终点
        if self.is_goal_reached():
            self.publish_cmd(0.0, 0.0)
            self.global_path = None
            self.get_logger().info('Goal reached')
            return

        self.publish_cmd(v, w)

    # ===============================
    # Lookahead 选择
    # ===============================
    def select_lookahead(self):
        rx, ry = self.robot_pose.x, self.robot_pose.y
        for i in range(self.target_idx, len(self.global_path)):
            p = self.global_path[i].pose.position
            if math.hypot(p.x - rx, p.y - ry) > self.lookahead_dist:
                self.target_idx = i
                return p
        return self.global_path[-1].pose.position

    # ===============================
    # Utils
    # ===============================
    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = clamp(v, 0.0, self.max_linear)
        cmd.angular.z = clamp(w, -self.max_angular, self.max_angular)
        self.cmd_pub.publish(cmd)

    def is_goal_reached(self):
        goal = self.global_path[-1].pose.position
        return math.hypot(
            goal.x - self.robot_pose.x,
            goal.y - self.robot_pose.y
        ) < self.goal_tolerance

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = RegulatedPurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
