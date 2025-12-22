#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math
import time


class DynamicObstacleMarkers(Node):

    def __init__(self):
        super().__init__('dynamic_obstacle_markers')

        # ============================
        # Publisher
        # ============================
        self.pub = self.create_publisher(
            MarkerArray,
            '/dynamic_obstacles_markers',
            10
        )

        # ============================
        # 地图尺寸（与你的 map 一致）
        # ============================
        self.map_width = 670 * 0.0165
        self.map_height = 669 * 0.0165

        # ============================
        # 动态障碍定义（与 dynamic_map 同源）
        # ============================

        # 横向贯穿
        self.obs1 = {
            'x': 0.5,
            'y': self.map_height / 2.0,
            'vx': 0.3,
            'vy': 0.0,
            'r': 0.25
        }

        # 纵向贯穿
        self.obs2 = {
            'x': self.map_width / 2.0,
            'y': 0.5,
            'vx': 0.0,
            'vy': 0.25,
            'r': 0.25
        }

        # 环形长路径
        self.obs3 = {
            'cx': self.map_width / 2.0,
            'cy': self.map_height / 2.0,
            'R': 4.0,
            'theta': 0.0,
            'omega': 0.15,
            'r': 0.3
        }

        self.last_time = time.time()

        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info('Dynamic obstacle Marker publisher started')

    # =====================================================
    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        markers = MarkerArray()

        # ========== 障碍 1 ==========
        self.obs1['x'] += self.obs1['vx'] * dt
        if self.obs1['x'] < 0.5 or self.obs1['x'] > self.map_width - 0.5:
            self.obs1['vx'] *= -1

        markers.markers += self.make_obstacle(
            0, self.obs1['x'], self.obs1['y'], self.obs1['r'], (1.0, 0.0, 0.0)
        )
        markers.markers.append(
            self.make_trajectory(
                10, self.linear_trajectory(self.obs1, 10.0),
                (1.0, 0.0, 0.0)
            )
        )

        # ========== 障碍 2 ==========
        self.obs2['y'] += self.obs2['vy'] * dt
        if self.obs2['y'] < 0.5 or self.obs2['y'] > self.map_height - 0.5:
            self.obs2['vy'] *= -1

        markers.markers += self.make_obstacle(
            1, self.obs2['x'], self.obs2['y'], self.obs2['r'], (0.0, 1.0, 0.0)
        )
        markers.markers.append(
            self.make_trajectory(
                11, self.linear_trajectory(self.obs2, 10.0),
                (0.0, 1.0, 0.0)
            )
        )

        # ========== 障碍 3（环形） ==========
        self.obs3['theta'] += self.obs3['omega'] * dt
        x3 = self.obs3['cx'] + self.obs3['R'] * math.cos(self.obs3['theta'])
        y3 = self.obs3['cy'] + self.obs3['R'] * math.sin(self.obs3['theta'])

        markers.markers += self.make_obstacle(
            2, x3, y3, self.obs3['r'], (0.0, 0.5, 1.0)
        )
        markers.markers.append(
            self.make_trajectory(
                12, self.circular_trajectory(self.obs3, 80),
                (0.0, 0.5, 1.0)
            )
        )

        self.pub.publish(markers)

    # =====================================================
    # Marker helpers
    # =====================================================
    def make_obstacle(self, idx, x, y, r, color):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'obstacles'
        m.id = idx
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.15
        m.pose.orientation.w = 1.0
        m.scale.x = r * 2
        m.scale.y = r * 2
        m.scale.z = 0.3
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        return [m]

    def make_trajectory(self, idx, points, color):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'trajectories'
        m.id = idx
        m.type = Marker.LINE_STRIP
        m.scale.x = 0.05
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 0.4
        for x, y in points:
            p = Point(x=x, y=y, z=0.05)
            m.points.append(p)
        return m

    # =====================================================
    # Trajectory prediction
    # =====================================================
    def linear_trajectory(self, obs, T):
        pts = []
        x, y = obs['x'], obs['y']
        vx, vy = obs['vx'], obs['vy']
        for i in range(50):
            t = T * i / 50.0
            pts.append((x + vx * t, y + vy * t))
        return pts

    def circular_trajectory(self, obs, N):
        pts = []
        for i in range(N):
            theta = obs['theta'] + 2 * math.pi * i / N
            x = obs['cx'] + obs['R'] * math.cos(theta)
            y = obs['cy'] + obs['R'] * math.sin(theta)
            pts.append((x, y))
        return pts


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
