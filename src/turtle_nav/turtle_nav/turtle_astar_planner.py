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


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('turtle_astar_planner')

        # =========================
        # Parameters
        # =========================
        self.declare_parameter('use_inflation', True)
        self.declare_parameter('use_costmap', True)
        self.declare_parameter('inflation_radius', 0.2)
        self.declare_parameter('cost_scaling', 10.0)
        self.declare_parameter('allow_unknown', False)

        self.use_inflation = self.get_parameter('use_inflation').value
        self.use_costmap = self.get_parameter('use_costmap').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.cost_scaling = self.get_parameter('cost_scaling').value
        self.allow_unknown = self.get_parameter('allow_unknown').value

        # =========================
        # QoS (关键修复)
        # =========================
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        goal_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # =========================
        # ROS I/O
        # =========================
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.path_pub = self.create_publisher(Path, '/plan', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # =========================
        # Map data
        # =========================
        self.map = None
        self.costmap = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None

        self.get_logger().info('A* planner started (shadow-forbidden mode)')

    # =====================================================
    # Map callback
    # =====================================================
    def map_callback(self, msg: OccupancyGrid):
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin.position

        raw = np.array(msg.data, dtype=np.int8).reshape(
            (self.height, self.width))

        # 核心规则：
        # 100  = obstacle
        # -1   = unknown（阴影）→ 当作 obstacle
        # 0    = free
        self.map = raw.copy()
        self.map[self.map < 0] = 100   # 🔒 永远禁止进入阴影

        if self.use_inflation:
            self.inflate_obstacles()

        if self.use_costmap:
            self.build_costmap()

        self.get_logger().info('Map received and processed')

    # =====================================================
    # Goal
    # =====================================================
    def goal_callback(self, goal: PoseStamped):
        self.get_logger().info(
            f'Received goal x={goal.pose.position.x:.2f}, '
            f'y={goal.pose.position.y:.2f}'
        )

        if self.map is None:
            self.get_logger().warn('No map yet')
            return

        start = self.get_robot_pose()
        if start is None:
            return

        start_idx = self.world_to_grid(start.x, start.y)
        goal_idx = self.world_to_grid(
            goal.pose.position.x, goal.pose.position.y)

        if not self.is_free(start_idx) or not self.is_free(goal_idx):
            self.get_logger().warn('Start or goal in obstacle / shadow')
            return

        path = self.astar(start_idx, goal_idx)
        if path:
            self.publish_path(path)

    # =====================================================
    # TF
    # =====================================================
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            return tf.transform.translation
        except Exception:
            self.get_logger().warn('TF not ready')
            return None

    # =====================================================
    # A*
    # =====================================================
    def astar(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}
        visited = set()

        while open_set:
            _, current = heapq.heappop(open_set)

            if current in visited:
                continue
            visited.add(current)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for n in self.neighbors(current):
                tentative = g_score[current] + self.cost(n)
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
    # Collision & cost
    # =====================================================
    def is_free(self, idx):
        x, y = idx
        return self.map[y, x] == 0

    def cost(self, idx):
        if self.costmap is None:
            return 1.0
        return 1.0 + self.costmap[idx[1], idx[0]]

    # =====================================================
    # Inflation
    # =====================================================
    def inflate_obstacles(self):
        r = int(self.inflation_radius / self.resolution)
        inflated = self.map.copy()

        obs = np.argwhere(self.map == 100)
        for oy, ox in obs:
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    ny, nx = oy + dy, ox + dx
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        inflated[ny, nx] = 100

        self.map = inflated

    # =====================================================
    # Costmap
    # =====================================================
    def build_costmap(self):
        self.costmap = np.zeros((self.height, self.width), dtype=np.float32)
        obs = np.argwhere(self.map == 100)

        for y in range(self.height):
            for x in range(self.width):
                dmin = min(
                    (math.hypot(x - ox, y - oy) for oy, ox in obs),
                    default=1e6
                )
                self.costmap[y, x] = math.exp(-dmin / self.cost_scaling)

    # =====================================================
    # Utils
    # =====================================================
    def world_to_grid(self, x, y):
        gx = int((x - self.origin.x) / self.resolution)
        gy = int((y - self.origin.y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        return (
            gx * self.resolution + self.origin.x,
            gy * self.resolution + self.origin.y
        )

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def publish_path(self, grid_path):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for gx, gy in grid_path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = \
                self.grid_to_world(gx, gy)
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
