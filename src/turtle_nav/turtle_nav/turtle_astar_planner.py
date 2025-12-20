#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import math
import heapq


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('turtle_astar_planner')

        # 订阅地图
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/static_map', self.map_callback, 10)

        # 订阅目标点（RViz 里点）
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # 发布路径
        self.path_pub = self.create_publisher(Path, '/plan', 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None

        self.get_logger().info('A* planner started')

    # =========================
    # Map
    # =========================
    def map_callback(self, msg: OccupancyGrid):
        self.map = msg.data
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin.position
        self.width = msg.info.width
        self.height = msg.info.height

    # =========================
    # Goal
    # =========================
    def goal_callback(self, goal: PoseStamped):
        if self.map is None:
            self.get_logger().warn('No map yet')
            return

        start = self.get_robot_pose()
        if start is None:
            return

        path = self.astar(start, goal.pose.position)
        if path:
            self.publish_path(path)

    # =========================
    # TF → robot pose
    # =========================
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            return tf.transform.translation
        except Exception:
            self.get_logger().warn('TF not ready')
            return None

    # =========================
    # A*
    # =========================
    def astar(self, start, goal):
        start_idx = self.world_to_grid(start.x, start.y)
        goal_idx = self.world_to_grid(goal.x, goal.y)

        open_set = []
        heapq.heappush(open_set, (0, start_idx))

        came_from = {}
        g_score = {start_idx: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_idx:
                return self.reconstruct_path(came_from, current)

            for n in self.neighbors(current):
                tentative = g_score[current] + 1
                if n not in g_score or tentative < g_score[n]:
                    came_from[n] = current
                    g_score[n] = tentative
                    f = tentative + self.heuristic(n, goal_idx)
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
                if self.map[ny * self.width + nx] == 0:
                    results.append((nx, ny))
        return results

    # =========================
    # Utils
    # =========================
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
