#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from operator import index
import rospy
import json
import time
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

class AutoDemo:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('auto_demo', anonymous=True)
        rospy.loginfo("=== TransBot Auto Demo Started ===")
        
        # Status tracking
        self.linefollow_active = False
        self.arm_active = False
        
        # Publishers (same as GUI)
        self.linefollow_control_pub = rospy.Publisher('/linefollow_control', String, queue_size=10)
        self.arm_cmd_pub = rospy.Publisher('/arm_cmd', String, queue_size=10)
        self.camera_cmd_pub = rospy.Publisher('/camera_cmd', String, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscribers for status monitoring
        self.linefollow_status_sub = rospy.Subscriber('/linefollow_status', String, self.linefollow_status_callback)
        self.arm_status_sub = rospy.Subscriber('/arm_status', String, self.arm_status_callback)
        
        rospy.loginfo("Auto demo initialization completed")
    
    def linefollow_status_callback(self, msg):
        """Monitor linefollow status"""
        try:
            status_data = json.loads(msg.data)
            self.linefollow_active = status_data.get('linefollow_enabled', False)
        except:
            pass
    
    def arm_status_callback(self, msg):
        """Monitor arm status"""
        try:
            status_data = json.loads(msg.data)
            self.arm_active = status_data.get('arm_active', False)
        except:
            pass

    # ===== 基础功能函数 =====
    
    def wait(self, seconds):
        """等待指定秒数"""
        rospy.loginfo("Waiting for {} seconds...".format(seconds))
        rospy.sleep(seconds)
    
    def log_step(self, step_name):
        """记录步骤日志"""
        rospy.loginfo("=== {} ===".format(step_name))

    # ===== 巡线控制函数 =====
    
    def start_linefollow(self, speed=0.3):
        """开始巡线"""
        rospy.loginfo("Starting linefollow with speed: {}".format(speed))
        try:
            cmd_data = {"cmd": "start", "speed": speed}
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.linefollow_control_pub.publish(msg)
            rospy.loginfo("Linefollow started successfully")
            return True
        except Exception as e:
            rospy.logerr("Failed to start linefollow: {}".format(e))
            return False
    
    def stop_linefollow(self):
        """停止巡线"""
        rospy.loginfo("Stopping linefollow")
        try:
            cmd_data = {"cmd": "stop"}
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.linefollow_control_pub.publish(msg)
            rospy.loginfo("Linefollow stopped successfully")
            return True
        except Exception as e:
            rospy.logerr("Failed to stop linefollow: {}".format(e))
            return False

    # ===== 机器人移动控制函数 =====
    
    def move_forward(self, distance, speed=0.2):
        """向前移动指定距离（米）"""
        rospy.loginfo("Moving forward {} meters at speed {}".format(distance, speed))
        try:
            # 计算移动时间（简单估算：时间 = 距离 / 速度）
            move_time = distance / speed
            
            # 发布前进命令
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = 0.0
            
            # 持续发布命令
            start_time = rospy.get_time()
            rate = rospy.Rate(10)  # 10Hz
            
            while rospy.get_time() - start_time < move_time:
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            
            # 停止
            self.stop_robot()
            rospy.loginfo("Forward movement completed")
            return True
            
        except Exception as e:
            rospy.logerr("Failed to move forward: {}".format(e))
            return False
    
    def move_backward(self, distance, speed=0.2):
        """向后移动指定距离（米）"""
        rospy.loginfo("Moving backward {} meters at speed {}".format(distance, speed))
        try:
            move_time = distance / speed
            
            twist = Twist()
            twist.linear.x = -speed  # 负值表示后退
            twist.angular.z = 0.0
            
            start_time = rospy.get_time()
            rate = rospy.Rate(10)
            
            while rospy.get_time() - start_time < move_time:
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            
            self.stop_robot()
            rospy.loginfo("Backward movement completed")
            return True
            
        except Exception as e:
            rospy.logerr("Failed to move backward: {}".format(e))
            return False
    
    def turn_left(self, angle_degrees, angular_speed=0.5):
        """左转指定角度（度）"""
        rospy.loginfo("Turning left {} degrees at angular speed {}".format(angle_degrees, angular_speed))
        try:
            # 角度转弧度
            angle_radians = angle_degrees * 3.14159 / 180.0
            # 计算转向时间
            turn_time = angle_radians / angular_speed
            
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = angular_speed  # 正值表示左转
            
            start_time = rospy.get_time()
            rate = rospy.Rate(10)
            
            while rospy.get_time() - start_time < turn_time:
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            
            self.stop_robot()
            rospy.loginfo("Left turn completed")
            return True
            
        except Exception as e:
            rospy.logerr("Failed to turn left: {}".format(e))
            return False
    
    def turn_right(self, angle_degrees, angular_speed=0.5):
        """右转指定角度（度）"""
        rospy.loginfo("Turning right {} degrees at angular speed {}".format(angle_degrees, angular_speed))
        try:
            angle_radians = angle_degrees * 3.14159 / 180.0
            turn_time = angle_radians / angular_speed
            
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = -angular_speed  # 负值表示右转
            
            start_time = rospy.get_time()
            rate = rospy.Rate(10)
            
            while rospy.get_time() - start_time < turn_time:
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            
            self.stop_robot()
            rospy.loginfo("Right turn completed")
            return True
            
        except Exception as e:
            rospy.logerr("Failed to turn right: {}".format(e))
            return False
    
    def stop_robot(self):
        """停止机器人移动"""
        rospy.loginfo("Stopping robot")
        try:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            rospy.loginfo("Robot stopped")
            return True
        except Exception as e:
            rospy.logerr("Failed to stop robot: {}".format(e))
            return False

    # ===== 摄像头云台控制函数 =====
    
    def camera_to_linefollow_position(self):
        """摄像头移动到巡线位置"""
        rospy.loginfo("Setting camera to linefollow position")
        try:
            cmd_data = {"cmd": "linefollow_position"}
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.camera_cmd_pub.publish(msg)
            rospy.loginfo("Camera moved to linefollow position")
            return True
        except Exception as e:
            rospy.logerr("Failed to set camera position: {}".format(e))
            return False
    
    def camera_to_position(self, pan_angle, tilt_angle):
        """摄像头移动到指定位置 (pan, tilt)"""
        rospy.loginfo("Setting camera to position: pan={}, tilt={}".format(pan_angle, tilt_angle))
        try:
            cmd_data = {
                "cmd": "set_position",
                "pan": pan_angle,
                "tilt": tilt_angle
            }
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.camera_cmd_pub.publish(msg)
            rospy.loginfo("Camera moved to position: pan={}, tilt={}".format(pan_angle, tilt_angle))
            return True
        except Exception as e:
            rospy.logerr("Failed to set camera position: {}".format(e))
            return False
    
    def camera_to_center(self):
        """摄像头回到中央位置"""
        rospy.loginfo("Setting camera to center position")
        return self.camera_to_position(90, 90)  # 假设90度为中央位置

    # ===== 机械臂控制函数 =====
    
    def arm_enable(self):
        """启用机械臂"""
        rospy.loginfo("Enabling arm")
        try:
            cmd_data = {"cmd": "enable"}
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.arm_cmd_pub.publish(msg)
            rospy.loginfo("Arm enabled successfully")
            return True
        except Exception as e:
            rospy.logerr("Failed to enable arm: {}".format(e))
            return False
    
    def arm_disable(self):
        """禁用机械臂"""
        rospy.loginfo("Disabling arm")
        try:
            cmd_data = {"cmd": "disable"}
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.arm_cmd_pub.publish(msg)
            rospy.loginfo("Arm disabled successfully")
            return True
        except Exception as e:
            rospy.logerr("Failed to disable arm: {}".format(e))
            return False
    
    def arm_move_to(self, motor1, motor2, motor3, motor4=None):
        """机械臂移动到指定位置"""
        if motor4 is None:
            motors = [motor1, motor2, motor3]
            rospy.loginfo("Moving arm to position: [{}, {}, {}]".format(motor1, motor2, motor3))
        else:
            motors = [motor1, motor2, motor3, motor4]
            rospy.loginfo("Moving arm to position: [{}, {}, {}, {}]".format(motor1, motor2, motor3, motor4))
        
        try:
            cmd_data = {
                "cmd": "move_motors",
                "motors": motors
            }
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.arm_cmd_pub.publish(msg)
            rospy.loginfo("Arm movement command sent successfully")
            return True
        except Exception as e:
            rospy.logerr("Failed to move arm: {}".format(e))
            return False
    
    def gripper_open(self):
        """张开爪子"""
        rospy.loginfo("Opening gripper")
        return self.arm_move_to(150, 100, 20, 50)  # 第4个参数控制爪子
    
    def gripper_close(self):
        """闭合爪子"""
        rospy.loginfo("Closing gripper")
        return self.arm_move_to(150, 100, 120, 200)  # 第4个参数控制爪子

    # ===== 组合动作函数 =====
    
    def linefollow_demo(self, duration=3.0, speed=0.1):
        """巡线演示"""
        self.log_step("Linefollow Demo")
        self.start_linefollow(speed)
        self.wait(duration)
        self.stop_linefollow()
    
    def arm_pick_demo(self):
        """机械臂抓取演示"""
        self.log_step("Arm Pick Demo")
       
        # 移动到抓取位置并张开爪子
        self.arm_move_to(100, 180, 20)
        self.wait(2)
        
        # 移动到抓取位置并闭合爪子
        self.arm_move_to(100, 180, 120)
        self.wait(3)
        
        
        # 移动到放置位置
        self.arm_move_to(170, 120, 120)
        self.wait(1)
        
        
        
    def arm_place_demo(self):
        """机械臂抓取演示"""
        self.log_step("Arm  Place Demo")
        
        # 移动到抓取位置并闭合爪子
        self.arm_move_to(150, 100, 120)
        self.wait(2)
        
        # 移动到抓取位置并张开爪子
        self.arm_move_to(150, 100, 20)
        self.wait(2)
        
        # 移动到放置位置
        self.arm_move_to(170, 120, 120)
        self.wait(2)
        
        

    def movement_demo(self):
        """移动演示"""
        self.log_step("Movement Demo")
        self.move_forward(0.5)  # 前进0.5米
        self.wait(1)
        self.turn_right(90)     # 右转90度
        self.wait(1)
        self.move_forward(0.3)  # 前进0.3米
        self.wait(1)
        self.turn_left(90)      # 左转90度
        self.wait(1)
        self.move_backward(0.3) # 后退0.3米

    def camera_demo(self):
        """摄像头演示"""
        self.log_step("Camera Demo")
        self.camera_to_position(45, 45)   # 左上
        self.wait(2)
        self.camera_to_position(135, 45)  # 右上
        self.wait(2)
        self.camera_to_position(135, 135) # 右下
        self.wait(2)
        self.camera_to_position(45, 135)  # 左下
        self.wait(2)
        self.camera_to_center()           # 回中央

    # ===== 主要演示序列 =====
    def back(self, building_index):
        if building_index in ["a"]:
            self.turn_left(60)
            self.linefollow_demo(duration=30.0, speed=0.13)
            self.turn_left(50)
        elif building_index in ["b"]:
            self.turn_right(55)
            self.move_forward(0.1,speed=0.1)
            self.linefollow_demo(duration=15.0, speed=0.15)
            self.move_forward(0.25,speed=0.1)
            self.turn_right(50)
        elif building_index in ["c"]:
            self.turn_right(57)
            self.move_forward(0.2,speed=0.1)
            self.linefollow_demo(duration=17.0, speed=0.15)
            self.move_forward(0.6,speed=0.1)
            self.linefollow_demo(duration=15.0, speed=0.15)
            self.move_forward(0.25,speed=0.1)
            self.turn_right(50)
        else:
            pass
    
    def write_scan(self, i):
        with open('/root/transbot_ws/src/transbot_composite_app/temp/scan.txt', 'w') as f:
            f.write(str(i))
            
    def write_end(self, i):
        with open('/root/transbot_ws/src/transbot_composite_app/temp/end.txt', 'w') as f:
            f.write(str(i))

    def write_a_b_c(self, s):
        with open('/root/transbot_ws/src/transbot_composite_app/temp_in/a_b_c.txt', 'w') as f:
            f.write(s)
    
    def write_left_right(self, s):
        with open('/root/transbot_ws/src/transbot_composite_app/temp_in/left_right_fire_cons.txt', 'w') as f:
            f.write(s)
    
    def read_a_b_c(self):
        with open('/root/transbot_ws/src/transbot_composite_app/temp_in/a_b_c.txt', 'r') as f:
            return f.read()
    
    def read_left_right(self):
        with open('/root/transbot_ws/src/transbot_composite_app/temp_in/left_right_fire_cons.txt', 'r') as f:
            return f.read()

    def run_demo(self):
        """执行完整的自动化演示流程 - 可以轻松调整顺序"""
        self.log_step("Auto Demo Sequence Started")
        
        try:
            self.camera_to_position(90, 60)
            self.write_end(0)
            self.write_scan(0)
            self.write_a_b_c("N")
            self.write_left_right("N")
            
            self.arm_enable()
            # 在这里可以轻松调整任务执行顺序
            
            # 移动到抓取位置
            self.arm_move_to(100, 180, 20)
            self.wait(3)
            
            # 巡线取外卖路上
            self.linefollow_demo(duration=11.0, speed=0.1)
            
            # 抓取外卖
            self.arm_pick_demo()
            self.write_end(1)
            
            self.write_scan(1)
            
            while(self.read_a_b_c() == "N"):
                self.wait(1)
                
            self.write_scan(0)
            
            #右转
            self.turn_right(50)
            #self.move_forward(0.1,speed=0.1)
            
            building_index = self.read_a_b_c()
            direction = self.read_left_right()
            
            #到a楼
            if building_index in ["a", "b", "c"]:
                self.move_forward(0.1,speed=0.1)
                self.linefollow_demo(duration=8.5, speed=0.1)
                
                
                
                
                #direction = self.read_left_right()
                
                
                
            #到b楼
            if building_index in ["b", "c"]:
                self.move_forward(0.1,speed=0.1)
                self.linefollow_demo(duration=14.0, speed=0.1)
                
                
                #direction = self.read_left_right()
                
            #到c楼
            if building_index in ["c"]:
                self.move_forward(0.1,speed=0.1)
                self.linefollow_demo(duration=35, speed=0.1)
                direction = None
            
            
            if building_index == "a":
                
                #self.write_scan(2)
                #self.wait(4)
                #self.write_scan(0)
                
                direction = "turn_left"
            elif building_index == "b":
                
                #self.write_scan(2)
                #self.wait(4)
                #self.write_scan(0)
                
                direction = "turn_right"
            elif building_index == "c":
                direction = None
            
            # 转向对应楼
            
            if direction == "turn_left":
                self.turn_left(50)
            elif direction == "turn_right":
                self.turn_right(43)
                self.linefollow_demo(duration=8.0, speed=0.1)
            else:
                pass

            #前往对应楼
            #self.move_forward(0.5,speed=0.1)
            self.linefollow_demo(duration=9.0, speed=0.1)
            
            # 放置外卖
            self.arm_place_demo()
            self.write_end(2)
            self.back(building_index)
            
            
            # 可选：添加其他演示
            # self.movement_demo()
            # self.wait(2)
            # self.camera_demo()
            self.arm_disable()
            self.log_step("Auto Demo Sequence Completed")
            
        except Exception as e:
            rospy.logerr("Demo sequence failed: {}".format(e))
            # 确保清理
            self.stop_robot()
            self.stop_linefollow()
            self.arm_disable()

def main():
    try:
        # Create auto demo
        demo = AutoDemo()
        
        # Wait for system to be ready
        rospy.loginfo("Waiting for system to be ready...")
        rospy.sleep(1.0)
        
        # Run the demo - 所有功能都可以一行调用
        demo.run_demo()
        
        # 也可以单独调用任何功能，例如：
        # demo.linefollow_demo(duration=5.0, speed=0.2)
        # demo.arm_pick_and_place_demo()
        # demo.movement_demo()
        # demo.camera_demo()
        
        rospy.loginfo("Demo completed, keeping node alive...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Auto demo interrupted by user")
    except Exception as e:
        rospy.logerr("Auto demo runtime error: {}".format(e))

if __name__ == '__main__':
    main() 