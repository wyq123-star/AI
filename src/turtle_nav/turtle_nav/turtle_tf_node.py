#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class TurtleTF(Node):

    def __init__(self):
        super().__init__('turtle_tf')

        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

    def pose_callback(self, msg: Pose):
        # 发布 odom
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = msg.y
        odom.pose.pose.orientation.z = math.sin(msg.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(msg.theta / 2.0)

        self.odom_pub.publish(odom)

        # 发布 TF
        tf = TransformStamped()
        tf.header.stamp = odom.header.stamp
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = msg.x
        tf.transform.translation.y = msg.y
        tf.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
