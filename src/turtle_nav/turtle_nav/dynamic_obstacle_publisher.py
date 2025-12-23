#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from geometry_msgs.msg import PoseArray, Pose as GeoPose


class DynamicObstacles(Node):

    def __init__(self):
        super().__init__('dynamic_obstacles')

        self.obstacle_names = ['obstacle1', 'obstacle2']
        self.obstacles = {name: None for name in self.obstacle_names}

        # Spawn service client
        self.spawn_cli = self.create_client(Spawn, '/spawn')
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')

        self.spawn_obstacles()

        # Subscribe pose
        for name in self.obstacle_names:
            self.create_subscription(
                Pose,
                f'/{name}/pose',
                lambda msg, n=name: self.pose_cb(msg, n),
                10
            )

        self.pub = self.create_publisher(
            PoseArray,
            '/dynamic_obstacles',
            10
        )

        self.timer = self.create_timer(0.05, self.publish)

        self.get_logger().info('Dynamic obstacles node started')

    def spawn_obstacles(self):
        positions = [(3.0, 5.0), (7.0, 5.0)]

        for name, (x, y) in zip(self.obstacle_names, positions):
            req = Spawn.Request()
            req.x = x
            req.y = y
            req.theta = 0.0
            req.name = name
            self.spawn_cli.call_async(req)
            self.get_logger().info(f'Spawned {name}')

    def pose_cb(self, msg, name):
        self.obstacles[name] = msg

    def publish(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for obs in self.obstacles.values():
            if obs is None:
                continue

            p = GeoPose()
            p.position.x = obs.x
            p.position.y = obs.y
            p.orientation.z = math.sin(obs.theta / 2.0)
            p.orientation.w = math.cos(obs.theta / 2.0)
            msg.poses.append(p)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacles()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
