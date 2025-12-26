#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

# 基础消息类型
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray
# 可视化与颜色消息
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA 

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class RegulatedPurePursuit(Node):

    def __init__(self):
        super().__init__('regulated_pure_pursuit')

        # ===============================
        # 1. 核心运动参数
        # ===============================
        self.lookahead_dist = 0.8     # 前瞻距离
        self.max_linear = 2.0         # 最大线速度
        self.max_angular = 2.0        # 最大角速度
        self.k_yaw = 2.5              # 转向增益

        # 原地转向滞回逻辑参数
        self.rotate_enter = 0.7       # 误差大于此值开始原地转
        self.rotate_exit = 0.3        # 误差小于此值恢复正常行驶

        # 调速因子
        self.min_speed_ratio = 0.2    # 最小速度百分比
        self.goal_tolerance = 0.2     # 到达容差

        # ===============================
        # 2. 动态窗口参数 (3x3m)
        # ===============================
        self.window_size = 3.0 
        self.is_obstacle_detected = False

        # ===============================
        # 3. 状态变量
        # ===============================
        self.robot_pose = None
        self.global_path = None
        self.target_idx = 0
        self.rotate_in_place = False
        self.dynamic_obstacles = []

        # ===============================
        # 4. ROS 通信
        # ===============================
        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.create_subscription(Path, '/plan', self.path_cb, 10)
        self.create_subscription(PoseArray, '/dynamic_obstacles', self.obstacle_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/local_window_marker', 10)

        # 20Hz 循环控制
        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info('Local Planner initialized with APF-inspired monitoring window.')

    # ===============================
    # 回调函数
    # ===============================
    def pose_cb(self, msg: Pose):
        self.robot_pose = msg

    def path_cb(self, msg: Path):
        self.global_path = msg.poses
        self.target_idx = 0
        self.rotate_in_place = False
        self.get_logger().info(f'Received new path: {len(self.global_path)} pts')

    def obstacle_cb(self, msg: PoseArray):
        self.dynamic_obstacles = msg.poses

    # ===============================
    # 主循环逻辑
    # ===============================
    def update(self):
        if self.robot_pose is None:
            return

        # --- 第一步：感知与警报 ---
        self.check_window_collision()
        self.publish_window_marker()

        if self.is_obstacle_detected:
            # 终端 Warn 警报 (频率限制 1.0s)
            self.get_logger().warn(
                '!!! DANGER: Obstacle detected in 3x3 Monitoring Window !!!', 
                throttle_duration_sec=1.0
            )

        if not self.global_path:
            return

        # --- 第二步：规划策略选择 ---
        # 如果检测到障碍物，进入“避障限速模式”
        safety_limit = 0.2 if self.is_obstacle_detected else 1.0

        target = self.select_lookahead()
        if target is None:
            return

        # 计算角度误差
        dx = target.x - self.robot_pose.x
        dy = target.y - self.robot_pose.y
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.robot_pose.theta)

        # --- 第三步：运动执行 ---
        # 1. 原地转向逻辑 (带滞回)
        if self.rotate_in_place:
            if abs(yaw_error) < self.rotate_exit:
                self.rotate_in_place = False
            else:
                self.publish_cmd(0.0, self.k_yaw * yaw_error)
                return
        else:
            if abs(yaw_error) > self.rotate_enter:
                self.rotate_in_place = True
                self.publish_cmd(0.0, self.k_yaw * yaw_error)
                return

        # 2. RPP 前进控制逻辑
        w = clamp(self.k_yaw * yaw_error, -self.max_angular, self.max_angular)
        
        # 调速逻辑：转弯越急，速度越慢
        speed_ratio = max(self.min_speed_ratio, 1.0 - abs(w) / self.max_angular)
        
        # 最终速度 = 最大线速度 * 调速因子 * 安全限速因子
        v = self.max_linear * speed_ratio * safety_limit

        # 3. 终点检查
        if self.is_goal_reached():
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info('Goal Reached successfully.')
            self.global_path = None
            return

        self.publish_cmd(v, w)

    # ===============================
    # 窗口与感知工具
    # ===============================
    def check_window_collision(self):
            self.is_obstacle_detected = False
            half_size = self.window_size / 2.0

            for obs in self.dynamic_obstacles:
                dx = obs.position.x - self.robot_pose.x
                dy = obs.position.y - self.robot_pose.y
                
                cos_t = math.cos(-self.robot_pose.theta)
                sin_t = math.sin(-self.robot_pose.theta)
                
                local_x = dx * cos_t - dy * sin_t
                local_y = dx * sin_t + dy * cos_t

                # --- 调试打印开始 ---
                # 计算直线距离
                dist = math.hypot(dx, dy)
                # 无论是否在窗口内都打印，这样你就能看到数据是否进来了
                # 注意：正式运行时请注释掉，否则会刷屏
                self.get_logger().info(f'检测到障碍物: 距离={dist:.2f}m, 局部坐标=({local_x:.2f}, {local_y:.2f})', throttle_duration_sec=0.5)
                # --- 调试打印结束 ---

                if abs(local_x) < half_size and abs(local_y) < half_size:
                    self.is_obstacle_detected = True
                    # 一旦发现一个在范围内的，就跳出循环
                    break

    def publish_window_marker(self):
        """发布可视化矩形框"""
        marker = Marker()
        marker.header.frame_id = "map" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "local_monitoring"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # 位姿跟随机器人
        marker.pose.position.x = self.robot_pose.x
        marker.pose.position.y = self.robot_pose.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.z = math.sin(self.robot_pose.theta / 2.0)
        marker.pose.orientation.w = math.cos(self.robot_pose.theta / 2.0)

        # 比例
        marker.scale.x = self.window_size
        marker.scale.y = self.window_size
        marker.scale.z = 0.02
        
        # 颜色控制
        if self.is_obstacle_detected:
            # 红色警报
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.4)
        else:
            # 绿色安全
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.2)

        self.marker_pub.publish(marker)

    # ===============================
    # 路径与底层工具
    # ===============================
    def select_lookahead(self):
        """选择前瞻点"""
        rx, ry = self.robot_pose.x, self.robot_pose.y
        for i in range(self.target_idx, len(self.global_path)):
            p = self.global_path[i].pose.position
            if math.hypot(p.x - rx, p.y - ry) > self.lookahead_dist:
                self.target_idx = i
                return p
        return self.global_path[-1].pose.position if self.global_path else None

    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def is_goal_reached(self):
        if not self.global_path: return False
        goal = self.global_path[-1].pose.position
        return math.hypot(goal.x - self.robot_pose.x, goal.y - self.robot_pose.y) < self.goal_tolerance

    @staticmethod
    def normalize_angle(a):
        while a > math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = RegulatedPurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()