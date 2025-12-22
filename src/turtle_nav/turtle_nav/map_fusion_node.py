#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy
import numpy as np


class MapFusion(Node):

    def __init__(self):
        super().__init__('map_fusion')

        self.static_map = None      # 只缓存一次
        self.dynamic_map = None     # 持续更新

        # ========= QoS =========
        # 静态地图：必须 Transient Local（map_server 只发一次）
        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # 动态地图：普通实时话题
        dynamic_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # ========= Subscribers =========
        self.sub_static = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.static_cb,
            static_qos
        )

        self.sub_dynamic = self.create_subscription(
            OccupancyGrid,
            '/dynamic_map',
            self.dynamic_cb,
            dynamic_qos
        )

        # ========= Publisher =========
        self.pub = self.create_publisher(
            OccupancyGrid,
            '/planning_map',
            10
        )

        # ========= Timer =========
        # 周期性融合（20Hz）
        self.timer = self.create_timer(0.05, self.try_publish)

        self.get_logger().info('Map fusion node started')

    # ======================================================
    # Callbacks
    # ======================================================
    def static_cb(self, msg: OccupancyGrid):
        # 只接收一次静态地图
        if self.static_map is None:
            self.static_map = msg
            self.get_logger().info('Static map received and cached')

    def dynamic_cb(self, msg: OccupancyGrid):
        self.dynamic_map = msg

    # ======================================================
    # Fusion
    # ======================================================
    def try_publish(self):
        if self.static_map is None or self.dynamic_map is None:
            return

        # 转 numpy
        static = np.array(self.static_map.data, dtype=np.int16)
        dynamic = np.array(self.dynamic_map.data, dtype=np.int16)

        # 安全检查
        if static.shape != dynamic.shape:
            self.get_logger().warn('Static map and dynamic map size mismatch')
            return

        # Nav2 风格融合：动态障碍覆盖静态
        fused_data = np.maximum(static, dynamic)

        fused = OccupancyGrid()
        fused.header.stamp = self.get_clock().now().to_msg()
        fused.header.frame_id = self.static_map.header.frame_id
        fused.info = self.static_map.info
        fused.data = fused_data.tolist()

        self.pub.publish(fused)


def main(args=None):
    rclpy.init(args=args)
    node = MapFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
