#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import time
from std_msgs.msg import String, Bool, Int32
from transbot_msgs.msg import PWMServo

class CameraController:
    """
    摄像头角度控制器
    控制摄像头的俯仰角度（上下：55-150度）和水平角度（左右：0-180度）
    直接使用真实舵机角度值，无映射转换
    """
    
    # 巡线位置统一配置
    LINEFOLLOW_PITCH = 40    # 巡线俯仰角度
    LINEFOLLOW_YAW = 92      # 巡线水平角度
    
    def __init__(self):
        rospy.init_node('camera_controller', anonymous=True)
        rospy.loginfo("=== Camera Controller Started ===")
        
        # 摄像头状态 - 直接使用舵机角度
        self.current_pitch = 100    # 当前俯仰角度（上下，id=2）10-140度
        self.current_yaw = 90      # 当前水平角度（左右，id=1）0-180度
        self.target_pitch = self.LINEFOLLOW_PITCH     # 目标俯仰角度（适合巡线）
        self.target_yaw = self.LINEFOLLOW_YAW         # 目标水平角度（正前方）
        
        # 角度限制 - 根据transbot_driver.py中的实际限制
        self.pitch_min = 10        # 俯仰最小角度（driver限制）
        self.pitch_max = 140       # 俯仰最大角度（driver限制）
        self.yaw_min = 0           # 水平最小角度
        self.yaw_max = 180         # 水平最大角度
        
        self.is_moving = False     # 是否正在移动
        
        # Publishers - 使用正确的PWMServo话题
        self.status_pub = rospy.Publisher('/camera_status', String, queue_size=1)
        self.servo_pub = rospy.Publisher('/PWMServo', PWMServo, queue_size=1)
        
        # Subscribers
        self.camera_cmd_sub = rospy.Subscriber('/camera_cmd', String, self.camera_cmd_callback)
        self.camera_angle_sub = rospy.Subscriber('/camera_angle', Int32, self.camera_angle_callback)
        
        # 状态发布定时器
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        
        # 初始化摄像头到默认角度
        rospy.sleep(1.0)  # 等待系统初始化
        self.set_camera_pitch(self.target_pitch)
        self.set_camera_yaw(self.target_yaw)
        
        rospy.loginfo("Camera controller initialization completed")
    
    def camera_cmd_callback(self, msg):
        """处理摄像头命令"""
        try:
            rospy.loginfo("Received camera command: {}".format(msg.data))
            
            if not msg.data or not msg.data.strip():
                return
                
            cmd_data = json.loads(msg.data)
            cmd_type = cmd_data.get('cmd', '')
            
            if cmd_type == "set_pitch":
                angle = cmd_data.get('angle', 100)
                self.set_camera_pitch(angle)
            elif cmd_type == "set_yaw":
                angle = cmd_data.get('angle', self.target_yaw)
                self.set_camera_yaw(angle)
            elif cmd_type == "set_angles":
                pitch = cmd_data.get('pitch', 100)
                yaw = cmd_data.get('yaw', self.current_yaw)
                self.set_camera_angles(pitch, yaw)
            elif cmd_type == "reset":
                self.set_camera_angles(self.target_pitch, self.target_yaw)
            elif cmd_type == "up":
                step = cmd_data.get('step', 10)
                new_pitch = max(self.current_pitch - step, self.pitch_min)  # 角度减小，摄像头向上
                self.set_camera_pitch(new_pitch)
            elif cmd_type == "down":
                step = cmd_data.get('step', 10)
                new_pitch = min(self.current_pitch + step, self.pitch_max)  # 角度增大，摄像头向下
                self.set_camera_pitch(new_pitch)
            elif cmd_type == "left":
                step = cmd_data.get('step', 10)
                new_yaw = min(self.current_yaw + step, self.yaw_max)
                self.set_camera_yaw(new_yaw)
            elif cmd_type == "right":
                step = cmd_data.get('step', 10)
                new_yaw = max(self.current_yaw - step, self.yaw_min)
                self.set_camera_yaw(new_yaw)
            elif cmd_type == "linefollow_position":
                # 设置到适合巡线的角度
                self.set_camera_angles(self.target_pitch, self.target_yaw)
            elif cmd_type == "center":
                # 设置到中心位置
                center_pitch = (self.pitch_min + self.pitch_max) // 2  # 102度
                self.set_camera_angles(center_pitch, 90)
            else:
                rospy.logwarn("Unknown camera command: {}".format(cmd_type))
                
        except json.JSONDecodeError as e:
            rospy.logerr("JSON parsing failed: {}".format(e))
        except Exception as e:
            rospy.logerr("Error processing camera command: {}".format(e))
    
    def camera_angle_callback(self, msg):
        """直接角度设置回调（默认设置俯仰角）"""
        self.set_camera_pitch(msg.data)
    
    def set_camera_pitch(self, pitch_angle):
        """设置摄像头俯仰角度（上下，id=2）"""
        try:
            # 限制角度范围
            pitch_angle = max(self.pitch_min, min(pitch_angle, self.pitch_max))
            
            rospy.loginfo("Setting camera pitch to: {} degrees".format(pitch_angle))
            self.is_moving = True
            
            # 发送PWM舵机控制命令
            servo_msg = PWMServo()
            servo_msg.id = 2  # id=2 是上下舵机（俯仰）
            servo_msg.angle = pitch_angle
            self.servo_pub.publish(servo_msg)
            
            # 计算移动时间（根据角度差异）
            angle_diff = abs(pitch_angle - self.current_pitch)
            move_time = max(0.3, angle_diff * 0.01)  # 每度约10ms
            
            self.current_pitch = pitch_angle
            
            # 等待移动完成
            rospy.sleep(move_time)
            self.is_moving = False
            
            rospy.loginfo("Camera pitch set to: {} degrees".format(pitch_angle))
                
        except Exception as e:
            rospy.logerr("Failed to set camera pitch: {}".format(e))
            self.is_moving = False
    
    def set_camera_yaw(self, yaw_angle):
        """设置摄像头水平角度（左右，id=1）"""
        try:
            # 限制角度范围
            yaw_angle = max(self.yaw_min, min(yaw_angle, self.yaw_max))
            
            rospy.loginfo("Setting camera yaw to: {} degrees".format(yaw_angle))
            self.is_moving = True
            
            # 发送PWM舵机控制命令
            servo_msg = PWMServo()
            servo_msg.id = 1  # id=1 是左右舵机（水平）
            servo_msg.angle = yaw_angle
            self.servo_pub.publish(servo_msg)
            
            # 计算移动时间（根据角度差异）
            angle_diff = abs(yaw_angle - self.current_yaw)
            move_time = max(0.3, angle_diff * 0.01)  # 每度约10ms
            
            self.current_yaw = yaw_angle
            
            # 等待移动完成
            rospy.sleep(move_time)
            self.is_moving = False
            
            rospy.loginfo("Camera yaw set to: {} degrees".format(yaw_angle))
                
        except Exception as e:
            rospy.logerr("Failed to set camera yaw: {}".format(e))
            self.is_moving = False
    
    def set_camera_angles(self, pitch_angle, yaw_angle):
        """同时设置俯仰和水平角度"""
        try:
            rospy.loginfo("Setting camera angles: pitch={} degrees, yaw={} degrees".format(pitch_angle, yaw_angle))
            
            # 限制角度范围
            pitch_angle = max(self.pitch_min, min(pitch_angle, self.pitch_max))
            yaw_angle = max(self.yaw_min, min(yaw_angle, self.yaw_max))
            
            self.is_moving = True
            
            # 发送俯仰角度
            servo_msg = PWMServo()
            servo_msg.id = 2
            servo_msg.angle = pitch_angle
            self.servo_pub.publish(servo_msg)
            rospy.sleep(0.1)  # 短暂延迟
            
            # 发送水平角度
            servo_msg = PWMServo()
            servo_msg.id = 1
            servo_msg.angle = yaw_angle
            self.servo_pub.publish(servo_msg)
            rospy.sleep(0.1)  # 短暂延迟
            
            # 计算总移动时间
            pitch_diff = abs(pitch_angle - self.current_pitch)
            yaw_diff = abs(yaw_angle - self.current_yaw)
            max_diff = max(pitch_diff, yaw_diff)
            move_time = max(0.5, max_diff * 0.01)
            
            self.current_pitch = pitch_angle
            self.current_yaw = yaw_angle
            
            # 等待移动完成
            rospy.sleep(move_time)
            self.is_moving = False
            
            rospy.loginfo("Camera angles set to: pitch={} degrees, yaw={} degrees".format(pitch_angle, yaw_angle))
            
        except Exception as e:
            rospy.logerr("Failed to set camera angles: {}".format(e))
            self.is_moving = False
    
    def get_current_angles(self):
        """获取当前角度"""
        return {"pitch": self.current_pitch, "yaw": self.current_yaw}
    
    def is_camera_moving(self):
        """检查摄像头是否正在移动"""
        return self.is_moving
    
    def prepare_for_linefollow(self):
        """准备巡线模式（设置到最佳角度）"""
        rospy.loginfo("Preparing camera for line following...")
        self.set_camera_angles(self.target_pitch, self.target_yaw)
        return not self.is_moving  # 返回是否完成
    
    def publish_status(self, event):
        """发布摄像头状态"""
        status_data = {
            "current_pitch": self.current_pitch,
            "current_yaw": self.current_yaw,
            "target_pitch": self.target_pitch,
            "target_yaw": self.target_yaw,
            "is_moving": self.is_moving,
            "pitch_min": self.pitch_min,
            "pitch_max": self.pitch_max,
            "yaw_min": self.yaw_min,
            "yaw_max": self.yaw_max,
            "timestamp": rospy.Time.now().to_sec()
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)
    
    def shutdown_hook(self):
        """关闭清理"""
        rospy.loginfo("Shutting down camera controller...")
        # 可以选择将摄像头重置到中间位置
        # self.set_camera_angles(102, 90)

def main():
    try:
        controller = CameraController()
        
        # 注册关闭回调
        rospy.on_shutdown(controller.shutdown_hook)
        
        rospy.loginfo("Camera controller ready")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Camera controller interrupted by user")
    except Exception as e:
        rospy.logerr("Camera controller runtime error: {}".format(e))

if __name__ == '__main__':
    main() 