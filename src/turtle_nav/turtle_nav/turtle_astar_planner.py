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
        # 参数
        # =========================
        self.declare_parameter('use_inflation', True)
        self.declare_parameter('inflation_radius', 1.0)
        self.declare_parameter('lethal_radius', 0.3)
        self.declare_parameter('cost_scaling', 10.0)
        self.declare_parameter('cost_weight', 5.0)
        self.declare_parameter('allow_unknown', False)

        self.use_inflation = self.get_parameter('use_inflation').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.lethal_radius = self.get_parameter('lethal_radius').value
        self.cost_scaling = self.get_parameter('cost_scaling').value
        self.cost_weight = self.get_parameter('cost_weight').value
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
        # 状态
        # =========================
        self.map = None
        self.costmap = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None

        self.pending_goal = None
        self.costmap_ready = False
        self.path_planned = False  # 标记是否已经规划过路径

        # =========================
        # Timer
        # =========================
        self.timer = self.create_timer(0.1, self.try_plan)

        self.get_logger().info('A* global planner started (one-time planning)')

    # =========================
    # Map callback
    # =========================
    def map_callback(self, msg: OccupancyGrid):
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin.position

        raw_map = np.array(msg.data, dtype=np.int8).reshape(
            (self.height, self.width))
        self.map = raw_map.copy()

        if self.use_inflation:
            self.build_costmap()
            self.costmap_ready = True
            self.get_logger().info('Static costmap built')

    # =========================
    # Goal callback
    # =========================
    def goal_callback(self, goal: PoseStamped):
        self.pending_goal = goal
        self.path_planned = False  # 新目标，需要重新规划
        self.get_logger().info(
            f'New goal received: x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}'
        )

    # =========================
    # Timer planning
    # =========================
    def try_plan(self):
        if self.pending_goal is None or self.map is None or self.path_planned:
            return

        start = self.get_robot_pose()
        if start is None:
            return

        goal = self.pending_goal
        self.pending_goal = None

        start_idx = self.world_to_grid(start.x, start.y)
        goal_idx = self.world_to_grid(goal.pose.position.x, goal.pose.position.y)

        path = self.astar(start_idx, goal_idx,
                          self.costmap if self.costmap_ready else None)
        if path:
            self.publish_path(path)
            self.path_planned = True
            self.get_logger().info(f'Global path published ({len(path)} points)')
        else:
            self.get_logger().warn('Failed to plan global path')

    # =========================
    # TF
    # =========================
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            return tf.transform.translation
        except Exception:
            return None

    # =========================
    # A* 核心算法（保持不变）
    # =========================
    def astar(self, start, goal, costmap=None):
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for n, step_cost in self.neighbors(current):
                tentative = g_score[current] + step_cost * self.cost(n, costmap)

                if n not in g_score or tentative < g_score[n]:
                    came_from[n] = current
                    g_score[n] = tentative
                    f = tentative + self.heuristic(n, goal)
                    heapq.heappush(open_set, (f, n))

        self.get_logger().warn('No path found')
        return None

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def neighbors(self, idx):
        x, y = idx
        results = []

        for dx, dy in [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if self.is_free((nx, ny)):
                    step = math.hypot(dx, dy)
                    results.append(((nx, ny), step))
        return results

    def is_free(self, idx):
        x, y = idx

        if self.map[y, x] == 100:
            return False

        if self.costmap_ready and not np.isfinite(self.costmap[y, x]):
            return False

        if self.map[y, x] == -1:
            return self.allow_unknown

        return True

    def cost(self, idx, costmap=None):
        if costmap is None:
            return 1.0
        return 1.0 + self.cost_weight * costmap[idx[1], idx[0]]

    # =========================
    # Costmap (静态膨胀)
    # =========================
    def build_costmap(self):
        obstacle_mask = (self.map == 100)
        dist = distance_transform_edt(~obstacle_mask) * self.resolution
        self.costmap = np.zeros_like(dist, dtype=np.float32)

        lethal_mask = dist < self.lethal_radius
        self.costmap[lethal_mask] = np.inf

        inflation_mask = (dist >= self.lethal_radius) & (dist <= self.inflation_radius)
        self.costmap[inflation_mask] = self.cost_scaling * np.exp(
            -(dist[inflation_mask] - self.lethal_radius)
        )

    # =========================
    # 工具函数
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


if __name__ == '__main__':
    main()
