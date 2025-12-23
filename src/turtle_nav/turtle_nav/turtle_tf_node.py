#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class TurtleTF(Node):

    def __init__(self):
        super().__init__('turtle_tf')

        self.tf_broadcaster = TransformBroadcaster(self)

        # robot
        self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.robot_cb,
            10
        )

        # obstacles
        self.obstacle_names = ['obstacle1', 'obstacle2']
        for name in self.obstacle_names:
            self.create_subscription(
                Pose,
                f'/{name}/pose',
                lambda msg, n=name: self.obstacle_cb(msg, n),
                10
            )

    def robot_cb(self, msg):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = msg.x
        tf.transform.translation.y = msg.y
        tf.transform.rotation.z = math.sin(msg.theta / 2.0)
        tf.transform.rotation.w = math.cos(msg.theta / 2.0)
        self.tf_broadcaster.sendTransform(tf)

    def obstacle_cb(self, msg, name):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = f'{name}_link'
        tf.transform.translation.x = msg.x
        tf.transform.translation.y = msg.y
        tf.transform.rotation.z = math.sin(msg.theta / 2.0)
        tf.transform.rotation.w = math.cos(msg.theta / 2.0)
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
