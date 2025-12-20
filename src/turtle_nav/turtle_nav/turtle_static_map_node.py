#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import OccupancyGrid

RESOLUTION = 0.1
WORLD_SIZE = 11.0
MAP_SIZE = int(WORLD_SIZE / RESOLUTION)


def world_to_map(x, y):
    mx = int(x / RESOLUTION)
    my = int(y / RESOLUTION)
    mx = np.clip(mx, 0, MAP_SIZE - 1)
    my = np.clip(my, 0, MAP_SIZE - 1)
    return mx, my


class TurtleStaticMap(Node):

    def __init__(self):
        super().__init__('turtle_static_map')

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/static_map',
            1
        )

        # costmap[y, x]
        self.costmap = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int8)
        self.init_map()

        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info('Static map node started')

    def init_map(self):
        # 边界
        self.costmap[0, :] = 100
        self.costmap[-1, :] = 100
        self.costmap[:, 0] = 100
        self.costmap[:, -1] = 100

        # 中间墙
        x1, y1 = world_to_map(3.0, 5.0)
        x2, y2 = world_to_map(8.0, 5.5)
        self.costmap[y1:y2, x1:x2] = 100

        # 圆形障碍
        cx, cy = world_to_map(6.0, 2.5)
        r = int(0.5 / RESOLUTION)
        for y in range(MAP_SIZE):
            for x in range(MAP_SIZE):
                if (x - cx)**2 + (y - cy)**2 <= r*r:
                    self.costmap[y, x] = 100

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = RESOLUTION
        msg.info.width = MAP_SIZE
        msg.info.height = MAP_SIZE
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.costmap.flatten().tolist()
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleStaticMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
