#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener


class PathFollower(Node):

    def __init__(self):
        super().__init__('turtle_path_follower')

        # 订阅路径
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)

        # 发布速度（turtlesim）
        self.cmd_pub = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path = None
        self.target_idx = 0

        # 参数（可以后面调）
        self.lookahead_dist = 0.5
        self.linear_speed = 1.5
        self.goal_tolerance = 0.2

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Path follower started')

    # =========================
    # Path callback
    # =========================
    def path_callback(self, msg: Path):
        self.path = msg.poses
        self.target_idx = 0
        self.get_logger().info(f'Received path with {len(self.path)} points')

    # =========================
    # Main control loop
    # =========================
    def control_loop(self):
        if not self.path:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        tx, ty = self.get_target_point(pose)
        dx = tx - pose['x']
        dy = ty - pose['y']

        # 角度误差
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - pose['yaw'])

        # 简单控制律
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = 2.5 * yaw_error

        # 到终点停止
        if self.is_goal_reached(pose):
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.path = None
            self.get_logger().info('Goal reached')

        self.cmd_pub.publish(cmd)

    # =========================
    # Utils
    # =========================
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            t = tf.transform.translation
            r = tf.transform.rotation

            yaw = math.atan2(
                2.0 * (r.w * r.z),
                1.0 - 2.0 * (r.z * r.z)
            )

            return {'x': t.x, 'y': t.y, 'yaw': yaw}
        except Exception:
            return None

    def get_target_point(self, pose):
        for i in range(self.target_idx, len(self.path)):
            px = self.path[i].pose.position.x
            py = self.path[i].pose.position.y
            if math.hypot(px - pose['x'], py - pose['y']) > self.lookahead_dist:
                self.target_idx = i
                return px, py
        return self.path[-1].pose.position.x, self.path[-1].pose.position.y

    def is_goal_reached(self, pose):
        goal = self.path[-1].pose.position
        return math.hypot(goal.x - pose['x'], goal.y - pose['y']) < self.goal_tolerance

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
