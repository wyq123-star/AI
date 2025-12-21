#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import heapq
import math
import numpy as np
from scipy.ndimage import distance_transform_edt


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('turtle_astar_planner')

        # =========================
        # Parameters
        # =========================
        self.declare_parameter('use_inflation', True)
        self.declare_parameter('inflation_radius', 0.2)
        self.declare_parameter('cost_scaling', 10.0)
        self.declare_parameter('allow_unknown', False)

        self.use_inflation = self.get_parameter('use_inflation').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.cost_scaling = self.get_parameter('cost_scaling').value
        self.allow_unknown = self.get_parameter('allow_unknown').value

        # =========================
        # ROS I/O
        # =========================
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        goal_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, goal_qos)

        self.path_pub = self.create_publisher(Path, '/plan', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # =========================
        # State
        # =========================
        self.map = None
        self.costmap = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None

        self.pending_goal = None
        self.costmap_ready = False

        # =========================
        # Timer for async planning (Nav2 style)
        # =========================
        self.timer = self.create_timer(0.1, self.try_plan)  # 10Hz

        self.get_logger().info('Nav2-style A* global planner started')

    # =====================================================
    # Map callback
    # =====================================================
    def map_callback(self, msg: OccupancyGrid):
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin.position

        raw_map = np.array(msg.data, dtype=np.int8).reshape(
            (self.height, self.width))
        self.map = raw_map.copy()

        # ====================
        # 构建快速 costmap
        # ====================
        if self.use_inflation:
            self.build_costmap()
            self.costmap_ready = True
            self.get_logger().info('Costmap built')

    # =====================================================
    # Goal callback（缓存 goal）
    # =====================================================
    def goal_callback(self, goal: PoseStamped):
        self.pending_goal = goal
        self.get_logger().info(
            f'Goal cached: x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}'
        )

    # =====================================================
    # Timer: try planning
    # =====================================================
    def try_plan(self):
        if self.pending_goal is None or self.map is None:
            return

        start = self.get_robot_pose()
        if start is None:
            return

        goal = self.pending_goal
        self.pending_goal = None  # 防止重复规划

        start_idx = self.world_to_grid(start.x, start.y)
        goal_idx = self.world_to_grid(goal.pose.position.x, goal.pose.position.y)

        # costmap 未就绪时用空 costmap（保证可以先规划）
        path = self.astar(start_idx, goal_idx, self.costmap if self.costmap_ready else None)
        if path:
            self.publish_path(path)
            self.get_logger().info(f'Path published with {len(path)} points')

    # =====================================================
    # TF
    # =====================================================
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            return tf.transform.translation
        except Exception:
            return None

    # =====================================================
    # A*
    # =====================================================
    def astar(self, start, goal, costmap=None):
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for n in self.neighbors(current):
                tentative = g_score[current] + self.cost(n, costmap)
                if n not in g_score or tentative < g_score[n]:
                    came_from[n] = current
                    g_score[n] = tentative
                    f = tentative + self.heuristic(n, goal)
                    heapq.heappush(open_set, (f, n))

        self.get_logger().warn('No path found')
        return None

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def neighbors(self, idx):
        x, y = idx
        results = []
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if self.is_free((nx, ny)):
                    results.append((nx, ny))
        return results

    # =====================================================
    # Cost & collision
    # =====================================================
    def is_free(self, idx):
        x, y = idx
        cell = self.map[y, x]
        if cell == 0:
            return True
        if cell == -1:
            return self.allow_unknown
        return False

    def cost(self, idx, costmap=None):
        if costmap is None:
            return 1.0
        return 1.0 + costmap[idx[1], idx[0]]

    # =====================================================
    # Fast costmap using distance transform
    # =====================================================
    def build_costmap(self):
        obstacle_mask = (self.map == 100)
        if self.use_inflation:
            dist = distance_transform_edt(~obstacle_mask) * self.resolution
            self.costmap = np.exp(-dist / self.cost_scaling)
        else:
            self.costmap = np.zeros_like(self.map, dtype=np.float32)

    # =====================================================
    # Utils
    # =====================================================
    def world_to_grid(self, x, y):
        gx = int((x - self.origin.x) / self.resolution)
        gy = int((y - self.origin.y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.resolution + self.origin.x
        y = gy * self.resolution + self.origin.y
        return x, y

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def publish_path(self, grid_path):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for gx, gy in grid_path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = self.grid_to_world(gx, gy)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
