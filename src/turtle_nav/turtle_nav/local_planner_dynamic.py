#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import math
import numpy as np


class DynamicLocalPlanner(Node):

    def __init__(self):
        super().__init__('dynamic_local_planner')

        # ===============================
        # 参数
        # ===============================
        self.lookahead_dist = 1.2
        self.linear_speed = 1.2
        self.sim_time = 1.0
        self.sim_dt = 0.1

        self.angular_samples = [-1.2, -0.9, -0.6, -0.3,
                                0.0,
                                0.3, 0.6, 0.9, 1.2]

        # ===============================
        # 状态
        # ===============================
        self.robot_pose = None
        self.global_path = None
        self.dynamic_map = None

        # ===============================
        # ROS IO
        # ===============================
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.create_subscription(Path, '/plan', self.path_cb, 10)
        self.create_subscription(OccupancyGrid, '/dynamic_map', self.map_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info('Dynamic Local Planner started')

    # ======================================================
    def pose_cb(self, msg):
        self.robot_pose = msg

    def path_cb(self, msg):
        self.global_path = msg.poses

    def map_cb(self, msg):
        self.dynamic_map = msg

    # ======================================================
    def update(self):
        if self.robot_pose is None or \
           self.global_path is None or \
           self.dynamic_map is None:
            return

        target = self.select_lookahead()
        if target is None:
            return

        best_cost = float('inf')
        best_w = 0.0

        for w in self.angular_samples:
            cost = self.simulate(self.linear_speed, w, target)
            if cost < best_cost:
                best_cost = cost
                best_w = w

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = best_w
        self.cmd_pub.publish(cmd)

    # ======================================================
    # 从全局路径选一个局部目标
    # ======================================================
    def select_lookahead(self):
        rx, ry = self.robot_pose.x, self.robot_pose.y
        for p in self.global_path:
            dx = p.pose.position.x - rx
            dy = p.pose.position.y - ry
            if math.hypot(dx, dy) > self.lookahead_dist:
                return p.pose.position
        return None

    # ======================================================
    # 轨迹模拟 + 代价评估
    # ======================================================
    def simulate(self, v, w, target):
        x = self.robot_pose.x
        y = self.robot_pose.y
        theta = self.robot_pose.theta

        cost = 0.0
        t = 0.0

        while t < self.sim_time:
            x += v * math.cos(theta) * self.sim_dt
            y += v * math.sin(theta) * self.sim_dt
            theta += w * self.sim_dt

            cell = self.world_to_grid(x, y)
            if cell is None:
                return float('inf')

            c = self.dynamic_map.data[cell]
            if c >= 100:
                return float('inf')

            cost += c
            t += self.sim_dt

        # 距离路径目标的代价
        dx = target.x - x
        dy = target.y - y
        cost += 10.0 * math.hypot(dx, dy)

        return cost

    # ======================================================
    # 世界坐标 → 栅格索引
    # ======================================================
    def world_to_grid(self, x, y):
        info = self.dynamic_map.info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)

        if 0 <= gx < info.width and 0 <= gy < info.height:
            return gy * info.width + gx
        return None


def main(args=None):
    rclpy.init(args=args)
    node = DynamicLocalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
