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
        self.lookahead_dist = 0.8     
        self.max_linear = 2.0         
        self.max_angular = 2.0        
        self.k_yaw = 2.5              

        self.rotate_enter = 0.7       
        self.rotate_exit = 0.3        

        self.min_speed_ratio = 0.2    
        self.goal_tolerance = 0.2     

        # ===============================
        # 2. 动态窗口与避障参数
        # ===============================
        self.window_size = 3.0           # 监测窗口大小 (3x3m)
        self.stop_distance = 0.8         # 强制停止距离 (半宽，即前后左右0.8m)
        self.is_obstacle_detected = False # 是否进入减速区
        self.is_critical_stop = False     # 是否进入停止区

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

        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info('Local Planner initialized. Mode: Deceleration & Emergency Stop active.')

    def pose_cb(self, msg: Pose):
        self.robot_pose = msg

    def path_cb(self, msg: Path):
        self.global_path = msg.poses
        self.target_idx = 0
        self.rotate_in_place = False
        self.get_logger().info(f'Received new path: {len(self.global_path)} pts')

    def obstacle_cb(self, msg: PoseArray):
        self.dynamic_obstacles = msg.poses

    def update(self):
        if self.robot_pose is None:
            return

        # --- 第一步：感知与警报 ---
        self.check_window_collision()
        self.publish_window_marker()

        # 根据感知结果确定安全限速因子
        safety_limit = 1.0
        if self.is_critical_stop:
            safety_limit = 0.0  # 极其靠近，紧急停止
            self.get_logger().error('!!! CRITICAL: Obstacle too close! STOPPING !!!', throttle_duration_sec=1.0)
        elif self.is_obstacle_detected:
            safety_limit = 0.3  # 进入监测区，减速行驶 (30% 速度)
            self.get_logger().warn('!!! WARNING: Obstacle in window. Slowing down !!!', throttle_duration_sec=1.0)

        # 如果没有路径，发布停止命令并返回
        if not self.global_path:
            self.publish_cmd(0.0, 0.0)
            return

        # --- 第二步：目标点选择 ---
        target = self.select_lookahead()
        if target is None:
            return

        dx = target.x - self.robot_pose.x
        dy = target.y - self.robot_pose.y
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.robot_pose.theta)

        # --- 第三步：运动执行 ---
        # 1. 原地转向逻辑 (此时受 safety_limit 影响较小，通常旋转不位移)
        if self.rotate_in_place:
            if abs(yaw_error) < self.rotate_exit:
                self.rotate_in_place = False
            else:
                # 即使原地转，如果太近也停止旋转
                rot_v = self.k_yaw * yaw_error if safety_limit > 0 else 0.0
                self.publish_cmd(0.0, rot_v)
                return
        else:
            if abs(yaw_error) > self.rotate_enter:
                self.rotate_in_place = True
                rot_v = self.k_yaw * yaw_error if safety_limit > 0 else 0.0
                self.publish_cmd(0.0, rot_v)
                return

        # 2. RPP 前进控制逻辑
        w = clamp(self.k_yaw * yaw_error, -self.max_angular, self.max_angular)
        speed_ratio = max(self.min_speed_ratio, 1.0 - abs(w) / self.max_angular)
        
        # 核心线速度公式
        v = self.max_linear * speed_ratio * safety_limit

        # 3. 终点检查
        if self.is_goal_reached():
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info('Goal Reached successfully.')
            self.global_path = None
            return

        self.publish_cmd(v, w)

    def check_window_collision(self):
        """检测并设置减速或停止标志"""
        self.is_obstacle_detected = False
        self.is_critical_stop = False
        
        half_size = self.window_size / 2.0 # 1.5m

        for obs in self.dynamic_obstacles:
            dx = obs.position.x - self.robot_pose.x
            dy = obs.position.y - self.robot_pose.y
            
            cos_t = math.cos(-self.robot_pose.theta)
            sin_t = math.sin(-self.robot_pose.theta)
            
            local_x = dx * cos_t - dy * sin_t
            local_y = dx * sin_t + dy * cos_t

            # A. 紧急停止判定 (更小的矩形范围)
            if abs(local_x) < self.stop_distance and abs(local_y) < self.stop_distance:
                self.is_critical_stop = True
                self.is_obstacle_detected = True
                break # 优先级最高，直接跳出
            
            # B. 减速警告判定 (大的 3x3 监测窗口)
            if abs(local_x) < half_size and abs(local_y) < half_size:
                self.is_obstacle_detected = True
                # 继续循环，看有没有更近的障碍物触发 critical_stop

    def publish_window_marker(self):
        marker = Marker()
        marker.header.frame_id = "map" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "safety_zones"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.robot_pose.x
        marker.pose.position.y = self.robot_pose.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.z = math.sin(self.robot_pose.theta / 2.0)
        marker.pose.orientation.w = math.cos(self.robot_pose.theta / 2.0)

        marker.scale.x = self.window_size
        marker.scale.y = self.window_size
        marker.scale.z = 0.02
        
        # 颜色分级可视化
        if self.is_critical_stop:
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7) # 深红 (停止)
        elif self.is_obstacle_detected:
            marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.4) # 橙色 (减速)
        else:
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.2) # 绿色 (安全)

        self.marker_pub.publish(marker)

    def select_lookahead(self):
        if not self.global_path: return None
        rx, ry = self.robot_pose.x, self.robot_pose.y
        for i in range(self.target_idx, len(self.global_path)):
            p = self.global_path[i].pose.position
            if math.hypot(p.x - rx, p.y - ry) > self.lookahead_dist:
                self.target_idx = i
                return p
        return self.global_path[-1].pose.position

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