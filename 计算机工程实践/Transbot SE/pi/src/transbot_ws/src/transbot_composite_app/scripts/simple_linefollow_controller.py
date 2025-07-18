#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import time
import yaml
import os
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from dynamic_reconfigure.client import Client

class SimpleLineFollowController:
    """
    简单巡线控制器
    控制何时转发巡线命令到机器人，并在开始前调整摄像头角度
    """
    def __init__(self):
        rospy.init_node('simple_linefollow_controller', anonymous=True)
        rospy.loginfo("=== Simple Line Follow Controller Started ===")
        
        # 加载PID参数配置
        self.load_pid_config()
        
        # 状态变量
        self.linefollow_enabled = False
        self.camera_ready = False
        self.preparing_camera = False
        self.linefollow_speed = self.pid_config.get('speed_parameters', {}).get('linear_speed', 0.25)
        
        # Dynamic reconfigure client for LineDetect
        # 延迟初始化，等待LineDetect节点启动
        self.dyn_client = None
        self.dyn_client_retry_count = 0
        self.max_dyn_client_retries = 5
        
        # 延迟初始化Dynamic Reconfigure客户端
        rospy.Timer(rospy.Duration(2.0), self.init_dynamic_reconfigure_client, oneshot=True)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/linefollow_status', String, queue_size=1)
        self.camera_cmd_pub = rospy.Publisher('/camera_cmd', String, queue_size=1)
        self.linefollow_cmd_pub = rospy.Publisher('/linefollow_cmd', String, queue_size=1)
        
        # Subscribers
        self.linefollow_cmd_sub = rospy.Subscriber('/cmd_vel_linefollow', Twist, self.linefollow_cmd_callback)
        self.control_sub = rospy.Subscriber('/linefollow_control', String, self.control_callback)
        self.camera_status_sub = rospy.Subscriber('/camera_status', String, self.camera_status_callback)
        
        # 状态发布定时器
        self.status_timer = rospy.Timer(rospy.Duration(0.2), self.publish_status)
        
        rospy.loginfo("Simple line follow controller initialized with PID config")
        self.log_pid_parameters()
    
    def load_pid_config(self):
        """加载PID参数配置文件"""
        try:
            # 获取配置文件路径
            package_path = rospy.get_param('~package_path', '/home/pi/src/transbot_ws/src/transbot_composite_app')
            config_file = os.path.join(package_path, 'config', 'linefollow_pid_params.yaml')
            
            rospy.loginfo("Loading PID config from: {}".format(config_file))
            
            if os.path.exists(config_file):
                with open(config_file, 'r', encoding='utf-8') as f:
                    self.pid_config = yaml.safe_load(f)
                rospy.loginfo("PID configuration loaded successfully")
            else:
                rospy.logwarn("PID config file not found: {}, using defaults".format(config_file))
                self.pid_config = self.get_default_pid_config()
                
        except Exception as e:
            rospy.logerr("Error loading PID config: {}, using defaults".format(e))
            self.pid_config = self.get_default_pid_config()
    
    def get_default_pid_config(self):
        """获取默认PID配置"""
        return {
            'pid_parameters': {
                'Kp': 20,
                'Ki': 0,
                'Kd': 6,
                'scale': 1000
            },
            'speed_parameters': {
                'linear_speed': 0.25,
                'max_angular_speed': 1.2,
                'min_angular_speed': 0.05
            },
            'error_processing': {
                'error_scale': 20,
                'image_center_x': 320,
                'dead_zone': 5
            },
            'debug': {
                'enable_pid_debug': True,
                'log_parameter_changes': True
            }
        }
    
    def log_pid_parameters(self):
        """记录当前PID参数"""
        pid_params = self.pid_config.get('pid_parameters', {})
        speed_params = self.pid_config.get('speed_parameters', {})
        
        rospy.loginfo("=== Current PID Parameters ===")
        rospy.loginfo("Kp: {}, Ki: {}, Kd: {}".format(
            pid_params.get('Kp', 20),
            pid_params.get('Ki', 0), 
            pid_params.get('Kd', 6)
        ))
        rospy.loginfo("Scale: {}".format(pid_params.get('scale', 1000)))
        rospy.loginfo("Linear Speed: {} m/s".format(speed_params.get('linear_speed', 0.25)))
        rospy.loginfo("Max Angular Speed: {} rad/s".format(speed_params.get('max_angular_speed', 1.2)))
        rospy.loginfo("===============================")
    
    def apply_pid_to_linedetect(self, pid_config):
        """通过Dynamic Reconfigure将PID参数应用到LineDetect节点"""
        try:
            rospy.loginfo("Applying PID parameters to LineDetect: {}".format(pid_config))
            rospy.loginfo("Current linefollow_speed value: {}".format(self.linefollow_speed))
            
            # 读取当前保存的HSV参数
            hsv_params = self.read_saved_hsv_params()
            
            # 准备Dynamic Reconfigure参数
            config = {
                'Kp': pid_config['Kp'],
                'Ki': pid_config['Ki'], 
                'Kd': pid_config['Kd'],
                'scale': pid_config['scale'],
                'linear': self.linefollow_speed
            }
            
            rospy.loginfo("Dynamic Reconfigure config with speed: {}".format(config))
            
            # 如果有HSV参数，一起设置
            if hsv_params:
                config.update({
                    'Hmin': hsv_params['hmin'],
                    'Smin': hsv_params['smin'],
                    'Vmin': hsv_params['vmin'],
                    'Hmax': hsv_params['hmax'],
                    'Smax': hsv_params['smax'],
                    'Vmax': hsv_params['vmax']
                })
                rospy.loginfo("Including HSV parameters in Dynamic Reconfigure: {}".format(hsv_params))
            else:
                rospy.logwarn("No saved HSV parameters found, using Dynamic Reconfigure defaults")
            
            rospy.loginfo("Final Dynamic Reconfigure config: {}".format(config))
            
            # 通过Dynamic Reconfigure设置参数
            self.dyn_client.update_configuration(config)
            rospy.loginfo("PID parameters successfully applied to LineDetect with speed: {}".format(config['linear']))
            
        except Exception as e:
            rospy.logerr("Failed to apply PID parameters to LineDetect: {}".format(e))
            raise e
    
    def read_saved_hsv_params(self):
        """读取保存的HSV参数"""
        try:
            import os
            
            # 优先从配置文件读取HSV参数，确保每次都是最新的
            workspace_root = "/root/transbot_ws"
            config_hsv_file = os.path.join(workspace_root, "src/transbot_composite_app/config/default_linefollow_hsv.txt")
            saved_hsv_file = os.path.join(workspace_root, "src/transbot_linefollow/scripts/default_linefollow_hsv.txt")
            
            # 优先读取配置文件
            if os.path.exists(config_hsv_file):
                with open(config_hsv_file, 'r') as f:
                    line = f.readline().strip()
                    if line:
                        values = [int(x.strip()) for x in line.split(',')]
                        if len(values) == 6:
                            hsv_params = {
                                'hmin': values[0],
                                'smin': values[1],
                                'vmin': values[2],
                                'hmax': values[3],
                                'smax': values[4],
                                'vmax': values[5]
                            }
                            rospy.loginfo("Read HSV parameters from config file: {}".format(hsv_params))
                            return hsv_params
            
           
            return None
            
        except Exception as e:
            rospy.logwarn("Failed to read HSV parameters: {}".format(e))
            return None
    
    def control_callback(self, msg):
        """处理巡线控制命令"""
        try:
            rospy.loginfo("Received linefollow control: {}".format(msg.data))
            
            if not msg.data or not msg.data.strip():
                return
                
            cmd_data = json.loads(msg.data)
            cmd_type = cmd_data.get('cmd', '')
            
            if cmd_type == "start":
                rospy.loginfo("Starting line following sequence...")
                # 保存速度参数
                self.linefollow_speed = cmd_data.get('speed', self.pid_config.get('speed_parameters', {}).get('linear_speed', 0.25))
                rospy.loginfo("Linefollow speed set to: {}".format(self.linefollow_speed))
                
                # 总是重新启动巡线序列，确保LineDetect收到最新命令
                if self.linefollow_enabled:
                    rospy.loginfo("Line following already enabled, restarting with new parameters...")
                else:
                    rospy.loginfo("Line following was disabled, starting...")
                
                self.start_linefollow_sequence()
                
            elif cmd_type == "stop":
                rospy.loginfo("Stopping line following")
                self.stop_linefollow()
                
            elif cmd_type == "set_speed":
                rospy.loginfo("Updating linefollow speed")
                if 'speed' in cmd_data:
                    self.linefollow_speed = cmd_data.get('speed', self.linefollow_speed)
                    rospy.loginfo("Linefollow speed updated to: {}".format(self.linefollow_speed))
                    
                    # 如果巡线正在运行，发送速度更新命令给LineDetect
                    if self.linefollow_enabled:
                        linefollow_cmd = {
                            "cmd": "set_speed",
                            "speed": self.linefollow_speed
                        }
                        linefollow_msg = String()
                        linefollow_msg.data = json.dumps(linefollow_cmd)
                        self.linefollow_cmd_pub.publish(linefollow_msg)
                        rospy.loginfo("Speed update command sent to LineDetect")
                        
                        # 只更新速度参数，但要保持HSV参数不变
                        if self.dyn_client:
                            try:
                                # 读取当前的HSV参数，避免被重置
                                current_hsv = self.read_saved_hsv_params()
                                
                                # 构建完整的配置，包含当前HSV参数
                                speed_config = {'linear': self.linefollow_speed}
                                
                                if current_hsv:
                                    speed_config.update({
                                        'Hmin': current_hsv['hmin'],
                                        'Smin': current_hsv['smin'],
                                        'Vmin': current_hsv['vmin'],
                                        'Hmax': current_hsv['hmax'],
                                        'Smax': current_hsv['smax'],
                                        'Vmax': current_hsv['vmax']
                                    })
                                    rospy.loginfo("Including current HSV parameters to preserve them: {}".format(current_hsv))
                                
                                self.dyn_client.update_configuration(speed_config)
                                rospy.loginfo("Speed updated via dynamic reconfigure (HSV parameters preserved)")
                            except Exception as e:
                                rospy.logwarn("Failed to update speed via dynamic reconfigure: {}".format(e))
                
            elif cmd_type == "reload_pid":
                rospy.loginfo("Reloading PID configuration")
                self.load_pid_config()
                self.log_pid_parameters()
                if self.linefollow_enabled:
                    self.apply_pid_to_linedetect(self.pid_config['pid_parameters'])
                    
            elif cmd_type == "status":
                # 状态查询，通过定时器自动发布
                pass
                
            else:
                rospy.logwarn("Unknown linefollow command: {}".format(cmd_type))
                
        except json.JSONDecodeError as e:
            rospy.logerr("JSON parsing failed: {}".format(e))
        except Exception as e:
            rospy.logerr("Error processing control command: {}".format(e))
    
    def start_linefollow_sequence(self):
        """启动巡线序列：重新加载配置，应用PID参数后启用巡线"""
        rospy.loginfo("=== Starting Line Follow Sequence with PID Parameters ===")
        
        # 重新加载PID配置，确保使用最新的参数
        rospy.loginfo("Reloading PID configuration from file...")
        self.load_pid_config()
        self.log_pid_parameters()
        
        # 首先应用PID参数到LineDetect节点（包含速度）
        try:
            self.apply_pid_to_linedetect(self.pid_config['pid_parameters'])
            rospy.loginfo("PID parameters applied successfully")
        except Exception as e:
            rospy.logwarn("Failed to apply PID parameters: {}, continuing with defaults".format(e))
        
        # 等待一小段时间确保参数生效
        time.sleep(0.5)
        
        # 直接启用巡线
        self.linefollow_enabled = True
        self.camera_ready = True  # 视为摄像头已就位
        self.preparing_camera = False
        
        # 发送启动命令给LineDetect节点，包含所有参数（在Dynamic Reconfigure之后）
        linefollow_cmd = {
            "cmd": "start",
            "speed": self.linefollow_speed,  # 优先使用用户设置的速度
            "pid_params": self.pid_config.get('pid_parameters', {}),
            "error_params": self.pid_config.get('error_processing', {})
        }
        linefollow_msg = String()
        linefollow_msg.data = json.dumps(linefollow_cmd)
        self.linefollow_cmd_pub.publish(linefollow_msg)
        
        rospy.loginfo("Line following ENABLED with speed: {} and custom PID parameters".format(self.linefollow_speed))
    
    def camera_status_callback(self, msg):
        """处理摄像头状态更新"""
        try:
            status_data = json.loads(msg.data)
            is_moving = status_data.get('is_moving', False)
            current_pitch = status_data.get('current_pitch', 0)
            current_yaw = status_data.get('current_yaw', 0)
            target_pitch = status_data.get('target_pitch', 40)
            target_yaw = status_data.get('target_yaw', 92)
            
            # 如果正在准备摄像头，检查是否完成
            if self.preparing_camera:
                # 检查俯仰角和水平角是否都到位
                pitch_ready = abs(current_pitch - target_pitch) < 2
                yaw_ready = abs(current_yaw - target_yaw) < 2
                
                if not is_moving and pitch_ready and yaw_ready:
                    # 摄像头已到位，启用巡线
                    self.camera_ready = True
                    self.preparing_camera = False
                    self.linefollow_enabled = True
                    
                    # 发送启动命令给LineDetect节点
                    linefollow_cmd = {
                        "cmd": "start",
                        "speed": self.linefollow_speed
                    }
                    linefollow_msg = String()
                    linefollow_msg.data = json.dumps(linefollow_cmd)
                    self.linefollow_cmd_pub.publish(linefollow_msg)
                    
                    rospy.loginfo("Camera ready at pitch={}, yaw={} degrees, line following ENABLED".format(
                        current_pitch, current_yaw))
                else:
                    rospy.loginfo("Camera adjusting: pitch={}→{}, yaw={}→{}, moving={}".format(
                        current_pitch, target_pitch, current_yaw, target_yaw, is_moving))
                        
        except json.JSONDecodeError as e:
            rospy.logdebug("Camera status JSON parsing failed: {}".format(e))
        except Exception as e:
            rospy.logdebug("Error processing camera status: {}".format(e))
    
    def stop_linefollow(self):
        """停止巡线"""
        self.linefollow_enabled = False
        self.camera_ready = False
        self.preparing_camera = False
        
        # 发送停止命令给LineDetect节点
        linefollow_cmd = {
            "cmd": "stop"
        }
        linefollow_msg = String()
        linefollow_msg.data = json.dumps(linefollow_cmd)
        self.linefollow_cmd_pub.publish(linefollow_msg)
        
        # 发送停止命令（零速度）
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        
        rospy.loginfo("Line following DISABLED")
    
    def linefollow_cmd_callback(self, msg):
        """处理来自巡线节点的速度命令"""
        if self.linefollow_enabled and self.camera_ready:
            # 转发命令到机器人
            self.cmd_vel_pub.publish(msg)
            rospy.logdebug("Forwarding linefollow command: linear={:.2f}, angular={:.2f}".format(
                msg.linear.x, msg.angular.z))
        else:
            # 巡线未启用，发送停止命令
            if not self.preparing_camera:  # 避免在准备阶段频繁日志
                rospy.logdebug("Line following disabled, ignoring command")
    
    def publish_status(self, event):
        """发布控制器状态"""
        status_data = {
            "linefollow_enabled": self.linefollow_enabled,
            "camera_ready": self.camera_ready,
            "preparing_camera": self.preparing_camera,
            "timestamp": rospy.Time.now().to_sec()
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)

    def init_dynamic_reconfigure_client(self, event):
        """初始化Dynamic Reconfigure客户端"""
        if self.dyn_client_retry_count < self.max_dyn_client_retries:
            try:
                self.dyn_client = Client("LineDetect", timeout=10)
                rospy.loginfo("Connected to LineDetect dynamic reconfigure server")
                return
            except Exception as e:
                rospy.logwarn("Failed to connect to LineDetect dynamic reconfigure: {}, retrying... ({}/{})".format(
                    e, self.dyn_client_retry_count + 1, self.max_dyn_client_retries))
                self.dyn_client_retry_count += 1
                rospy.Timer(rospy.Duration(2.0), self.init_dynamic_reconfigure_client, oneshot=True)
        else:
            rospy.logwarn("Max retries reached, dynamic reconfigure client initialization failed")

def main():
    try:
        controller = SimpleLineFollowController()
        rospy.loginfo("Simple line follow controller ready")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Simple line follow controller interrupted by user")
    except Exception as e:
        rospy.logerr("Simple line follow controller runtime error: {}".format(e))

if __name__ == '__main__':
    main() 