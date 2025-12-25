#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class ObstacleMotion(Node):

    def __init__(self):
        super().__init__('obstacle_motion')

        # ===============================
        # 每个动态障碍物的巡航路径（turtlesim 安全区）
        # ===============================
        self.paths = {
            'obstacle1': [(2.0, 2.0), (9.0, 2.0), (9.0, 9.0), (2.0, 9.0)],
            'obstacle2': [(3.0, 8.0), (8.0, 8.0), (8.0, 3.0), (3.0, 3.0)],
        }

        self.states = {
            name: {'idx': 0}
            for name in self.paths
        }

        self.poses = {}

        # ===============================
        # ROS IO
        # ===============================
        self.cmd_pubs = {
            name: self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            for name in self.paths
        }

        for name in self.paths:
            self.create_subscription(
                Pose,
                f'/{name}/pose',
                lambda msg, n=name: self.pose_cb(msg, n),
                10
            )

        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info('Obstacle motion controller started')

    # ======================================================
    def pose_cb(self, msg, name):
        self.poses[name] = msg

    # ======================================================
    def update(self):
        for name, path in self.paths.items():
            if name not in self.poses:
                continue

            pose = self.poses[name]
            state = self.states[name]
            target = path[state['idx']]

            dx = target[0] - pose.x
            dy = target[1] - pose.y
            dist = math.hypot(dx, dy)

            cmd = Twist()

            if dist < 0.25:
                state['idx'] = (state['idx'] + 1) % len(path)
            else:
                desired = math.atan2(dy, dx)
                err = self.angle_diff(desired, pose.theta)

                cmd.linear.x = 0.8
                cmd.angular.z = 4.0 * err

            self.cmd_pubs[name].publish(cmd)

    @staticmethod
    def angle_diff(a, b):
        d = a - b
        return math.atan2(math.sin(d), math.cos(d))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
