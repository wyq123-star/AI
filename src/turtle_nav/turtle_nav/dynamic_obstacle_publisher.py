#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import time


class PredictiveDynamicLayer(Node):

    def __init__(self):
        super().__init__('predictive_dynamic_layer')

        # ===============================
        # 地图参数（必须与你的静态地图一致）
        # ===============================
        self.resolution = 0.0165
        self.width = 670
        self.height = 669
        self.origin_x = 0.0
        self.origin_y = 0.0

        # ===============================
        # 预测参数（关键）
        # ===============================
        self.prediction_horizon = 3.0    # 预测 3 秒
        self.dt = 0.5                    # 时间分辨率
        self.base_radius = 0.25          # 障碍物半径
        self.time_decay = 1.2            # 时间衰减系数

        # ===============================
        # Publisher
        # ===============================
        self.pub = self.create_publisher(
            OccupancyGrid,
            '/dynamic_map',
            10
        )

        # ===============================
        # 动态地图
        # ===============================
        self.grid = np.zeros((self.height, self.width), dtype=np.int8)

        # ===============================
        # 已知轨迹的动态障碍（示例）
        # ===============================
        self.obstacles = [
            # 横向贯穿迷宫
            {'x': 0.5, 'y': 4.5, 'vx': 0.35, 'vy': 0.0},

            # 纵向贯穿迷宫
            {'x': 4.0, 'y': 0.5, 'vx': 0.0, 'vy': 0.30},

            # 圆形长路径（中心封堵）
            {'cx': 4.5, 'cy': 4.5, 'r': 3.8,
             'theta': 0.0, 'omega': 0.18}
        ]

        self.last_time = time.time()
        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info('Predictive Dynamic Layer started')

    # ======================================================
    # 主循环
    # ======================================================
    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        self.grid.fill(0)

        # -------- 障碍 1 / 2：线性运动 --------
        for obs in self.obstacles[:2]:
            obs['x'] += obs['vx'] * dt
            obs['y'] += obs['vy'] * dt

            self.predict_and_draw(
                lambda t, o=obs: (
                    o['x'] + o['vx'] * t,
                    o['y'] + o['vy'] * t
                )
            )

        # -------- 障碍 3：环形运动 --------
        ring = self.obstacles[2]
        ring['theta'] += ring['omega'] * dt

        self.predict_and_draw(
            lambda t, o=ring: (
                o['cx'] + o['r'] * math.cos(o['theta'] + o['omega'] * t),
                o['cy'] + o['r'] * math.sin(o['theta'] + o['omega'] * t)
            )
        )

        self.publish()

    # ======================================================
    # 预测 + 绘制
    # ======================================================
    def predict_and_draw(self, predict_fn):
        t = 0.0
        while t <= self.prediction_horizon:
            x, y = predict_fn(t)

            if t == 0.0:
                cost = 100  # 当前真实位置：致命
            else:
                cost = int(
                    100 * math.exp(-self.time_decay * t)
                )
                cost = max(20, min(cost, 90))

            self.draw_circle(x, y, self.base_radius, cost)
            t += self.dt

    # ======================================================
    # 栅格绘制
    # ======================================================
    def draw_circle(self, x, y, radius, cost):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        r = int(radius / self.resolution)

        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx*dx + dy*dy <= r*r:
                    ix, iy = gx + dx, gy + dy
                    if 0 <= ix < self.width and 0 <= iy < self.height:
                        self.grid[iy, ix] = max(self.grid[iy, ix], cost)

    # ======================================================
    # 发布
    # ======================================================
    def publish(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid.flatten().tolist()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PredictiveDynamicLayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
