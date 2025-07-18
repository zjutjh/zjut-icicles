#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import math
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
import json

class NavigationController:
    def __init__(self):
        rospy.init_node('navigation_controller', anonymous=True)
        rospy.loginfo("=== Navigation Controller Started ===")
        
        # 导航状态
        self.current_pose = None
        self.target_pose = None
        self.path_following = False
        self.obstacle_detected = False
        
        # 导航参数
        self.max_linear_vel = 0.5  # m/s
        self.max_angular_vel = 1.0  # rad/s
        self.goal_tolerance = 0.1  # m
        self.obstacle_distance = 0.3  # m
        
        # 发布器
        self.cmd_vel_pub = rospy.Publisher('/nav_cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('/navigation_path', Path, queue_size=1)
        self.status_pub = rospy.Publisher('/nav_status', String, queue_size=1)
        
        # 订阅器
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.nav_cmd_sub = rospy.Subscriber('/nav_cmd', String, self.nav_cmd_callback)
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 控制循环定时器
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
        rospy.loginfo("Navigation controller initialization completed")
    
    def odom_callback(self, msg):
        """里程计回调，更新当前位置"""
        self.current_pose = msg.pose.pose
        
    def goal_callback(self, msg):
        """目标点回调"""
        self.target_pose = msg.pose
        self.path_following = True
        rospy.loginfo("Received navigation goal: x={:.2f}, y={:.2f}".format(
            msg.pose.position.x, msg.pose.position.y))
        
    def laser_callback(self, msg):
        """激光雷达回调，检测障碍物"""
        if not msg.ranges:
            return
            
        # 检查前方扇形区域是否有障碍物
        front_ranges = []
        angle_increment = msg.angle_increment
        start_idx = int(len(msg.ranges) * 0.4)  # 前方72度范围
        end_idx = int(len(msg.ranges) * 0.6)
        
        for i in range(start_idx, end_idx):
            if msg.range_min <= msg.ranges[i] <= msg.range_max:
                front_ranges.append(msg.ranges[i])
        
        if front_ranges:
            min_distance = min(front_ranges)
            self.obstacle_detected = min_distance < self.obstacle_distance
        else:
            self.obstacle_detected = False
    
    def nav_cmd_callback(self, msg):
        """导航指令回调"""
        try:
            cmd_data = json.loads(msg.data)
            cmd_type = cmd_data.get('cmd', '')
            
            if cmd_type == "goto":
                # 接收目标坐标
                x = cmd_data.get('x', 0.0)
                y = cmd_data.get('y', 0.0)
                theta = cmd_data.get('theta', 0.0)
                self.set_navigation_goal(x, y, theta)
                
            elif cmd_type == "stop":
                self.stop_navigation()
                
            elif cmd_type == "resume":
                self.path_following = True
                
        except ValueError:  # JSON decode error in Python 2.7
            rospy.logerr("Navigation command JSON parsing failed: {}".format(msg.data))
        except Exception as e:
            rospy.logerr("Error processing navigation command: {}".format(e))
    
    def set_navigation_goal(self, x, y, theta=0.0):
        """设置导航目标"""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        
        # 转换角度为四元数
        goal.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.orientation.w = math.cos(theta / 2.0)
        
        self.target_pose = goal.pose
        self.path_following = True
        rospy.loginfo("Set navigation goal: ({:.2f}, {:.2f}, {:.2f})".format(x, y, theta))
    
    def stop_navigation(self):
        """停止导航"""
        self.path_following = False
        self.target_pose = None
        
        # 发送停止指令
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        rospy.loginfo("Navigation stopped")
    
    def control_loop(self, event):
        """主控制循环"""
        if not self.path_following or not self.target_pose or not self.current_pose:
            return
        
        # 计算到目标的距离和角度
        dx = self.target_pose.position.x - self.current_pose.position.x
        dy = self.target_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 检查是否到达目标
        if distance < self.goal_tolerance:
            self.stop_navigation()
            rospy.loginfo("Reached navigation goal")
            return
        
        # 计算目标角度
        target_angle = math.atan2(dy, dx)
        
        # 获取当前朝向
        current_orientation = self.current_pose.orientation
        current_angle = 2 * math.atan2(current_orientation.z, current_orientation.w)
        
        # 计算角度差
        angle_diff = target_angle - current_angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 生成控制指令
        twist = Twist()
        
        if self.obstacle_detected:
            # 简单的避障：后退并转向
            twist.linear.x = -0.1
            twist.angular.z = 0.5 if angle_diff > 0 else -0.5
            rospy.logwarn("Obstacle detected, performing avoidance")
        else:
            # 正常导航
            if abs(angle_diff) > 0.1:
                # 先转向目标
                twist.angular.z = max(-self.max_angular_vel, 
                                    min(self.max_angular_vel, 2.0 * angle_diff))
            else:
                # 朝向正确，前进
                twist.linear.x = max(0.0, min(self.max_linear_vel, distance))
                twist.angular.z = max(-self.max_angular_vel, 
                                    min(self.max_angular_vel, angle_diff))
        
        self.cmd_vel_pub.publish(twist)
        
        # 发布状态
        self.publish_navigation_status(distance, angle_diff)
    
    def publish_navigation_status(self, distance, angle_diff):
        """发布导航状态"""
        status_data = {
            "distance_to_goal": distance,
            "angle_to_goal": angle_diff,
            "path_following": self.path_following,
            "obstacle_detected": self.obstacle_detected,
            "timestamp": rospy.Time.now().to_sec()
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)

def main():
    try:
        nav_controller = NavigationController()
        rospy.loginfo("Navigation controller ready")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation controller interrupted by user")
    except Exception as e:
        rospy.logerr("Navigation controller runtime error: {}".format(e))

if __name__ == '__main__':
    main() 