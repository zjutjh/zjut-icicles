#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess
import os
import signal
import json
import time
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from dynamic_reconfigure.client import Client

class LineFollowController:
    def __init__(self):
        rospy.init_node('linefollow_controller', anonymous=True)
        rospy.loginfo("=== Line Follow Controller Started (calling transbot_linefollow package) ===")
        
        # Line following status
        self.linefollow_active = False
        self.linefollow_process = None
        self.dyn_client = None
        self.startup_timeout = 10.0  # 启动超时时间
        self.last_cmd_data = None   # 保存最后的启动参数
        
        # Publishers
        self.status_pub = rospy.Publisher('/linefollow_status', String, queue_size=1)
        
        # Subscribers
        self.follow_cmd_sub = rospy.Subscriber('/follow_cmd', String, self.follow_cmd_callback)
        self.linefollow_enable_sub = rospy.Subscriber('/linefollow_enable', Bool, self.enable_callback)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Status publishing timer
        self.status_timer = rospy.Timer(rospy.Duration(0.2), self.publish_status)
        
        # Recent cmd_vel information (for status monitoring)
        self.last_cmd_vel = Twist()
        
        rospy.loginfo("Line follow controller initialization completed")
    
    def enable_callback(self, msg):
        """Line following enable/disable callback"""
        if msg.data:
            self.start_linefollow()
        else:
            self.stop_linefollow()
    
    def follow_cmd_callback(self, msg):
        """Line following command callback"""
        try:
            rospy.loginfo("=== Received line follow command ===")
            rospy.loginfo("Raw message data: {}".format(msg.data))
            
            # 检查消息数据是否为空
            if not msg.data or not msg.data.strip():
                rospy.logerr("Empty command data received")
                return
                
            cmd_data = json.loads(msg.data)
            cmd_type = cmd_data.get('cmd', '')
            
            rospy.loginfo("Parsed command type: {}".format(cmd_type))
            rospy.logdebug("Full command data: {}".format(cmd_data))
            
            if cmd_type == "start":
                rospy.loginfo("Processing START command...")
                self.last_cmd_data = cmd_data  # 保存启动参数
                success = self.start_linefollow(cmd_data)
                if success:
                    rospy.loginfo("Line follow started successfully")
                else:
                    rospy.logerr("Failed to start line follow")
            elif cmd_type == "stop":
                rospy.loginfo("Processing STOP command...")
                success = self.stop_linefollow()
                if success:
                    rospy.loginfo("Line follow stopped successfully")
                else:
                    rospy.logerr("Failed to stop line follow")
            elif cmd_type == "set_params":
                rospy.loginfo("Processing SET_PARAMS command...")
                # Set line following parameters
                self.set_linefollow_params(cmd_data)
            elif cmd_type == "reset":
                rospy.loginfo("Processing RESET command...")
                # Reset line following
                success = self.reset_linefollow()
                if success:
                    rospy.loginfo("Line follow reset successfully")
                else:
                    rospy.logerr("Failed to reset line follow")
            else:
                rospy.logwarn("Unknown line follow command: {}".format(cmd_type))
                
        except json.JSONDecodeError as e:
            rospy.logerr("JSON parsing failed for message: {}".format(msg.data))
            rospy.logerr("JSON decode error: {}".format(e))
        except ValueError as e:  # For Python 2.7 compatibility
            rospy.logerr("JSON parsing failed for message: {}".format(msg.data))
            rospy.logerr("Value error: {}".format(e))
        except KeyError as e:
            rospy.logerr("Missing required key in command data: {}".format(e))
            rospy.logerr("Command data was: {}".format(msg.data))
        except Exception as e:
            rospy.logerr("Unexpected error processing line follow command: {}".format(e))
            import traceback
            rospy.logerr("Full traceback: {}".format(traceback.format_exc()))
            rospy.logerr("Problematic message: {}".format(msg.data))
    
    def start_linefollow(self, cmd_data=None):
        """Start line following function"""
        if self.linefollow_process is not None:
            rospy.logwarn("Line following already running, stopping first...")
            self.stop_linefollow()
            rospy.sleep(1.0)
            
        try:
            # 准备启动参数
            launch_args = []
            if cmd_data:
                # 提取参数并传递给launch文件
                img_flip = cmd_data.get('img_flip', False)
                video_switch = cmd_data.get('VideoSwitch', False)
                
                if img_flip:
                    launch_args.append("img_flip:=true")
                else:
                    launch_args.append("img_flip:=false")
                    
                if video_switch:
                    launch_args.append("VideoSwitch:=true")
                else:
                    launch_args.append("VideoSwitch:=false")
                    
                rospy.loginfo("Launch arguments: {}".format(launch_args))
            
            # Start transbot_linefollow package launch file
            launch_cmd_list = ["roslaunch", "transbot_linefollow", "follow_line.launch"] + launch_args
            launch_cmd_str = ' '.join(launch_cmd_list)

            # 构建一个能在新shell中执行的完整命令，确保环境被正确加载
            # 这是解决在subprocess中启动GUI应用的关键
            source_path = "/root/transbot_ws/devel/setup.bash"
            full_command = "bash -c 'source {} && {}'".format(source_path, launch_cmd_str)
            
            rospy.loginfo("Starting line follow with full shell command: " + full_command)
            
            # 使用 shell=True 来执行完整的命令字符串
            self.linefollow_process = subprocess.Popen(
                full_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                shell=True,
                preexec_fn=os.setsid
            )
            
            # 检查进程是否启动成功
            rospy.loginfo("Waiting for process startup...")
            start_time = time.time()
            process_started = False
            
            while time.time() - start_time < self.startup_timeout:
                if self.linefollow_process.poll() is not None:
                    # 进程已退出，启动失败
                    stdout, stderr = self.linefollow_process.communicate()
                    rospy.logerr("Line follow process failed to start:")
                    rospy.logerr("STDOUT: {}".format(stdout))
                    rospy.logerr("STDERR: {}".format(stderr))
                    self.linefollow_process = None
                    self.linefollow_active = False
                    return False
                
                # 检查是否有LineDetect节点启动
                try:
                    node_list = rospy.get_published_topics()
                    linedetect_topics = [topic for topic, _ in node_list if 'LineDetect' in topic or '/follow' in topic]
                    if linedetect_topics:
                        process_started = True
                        break
                except:
                    pass
                    
                rospy.sleep(0.5)
            
            if not process_started:
                rospy.logwarn("Process startup verification timed out, but process seems running")
            
            self.linefollow_active = True
            
            # Wait a bit more for full node startup
            rospy.sleep(2.0)
            
            # Initialize dynamic parameter client
            try:
                rospy.loginfo("Connecting to dynamic parameter server...")
                self.dyn_client = Client("LineDetect", timeout=10)
                rospy.loginfo("Dynamic parameter client connected successfully")
                
                # 应用参数（如果有）
                if cmd_data:
                    self.apply_startup_params(cmd_data)
                    
            except Exception as e:
                rospy.logwarn("Dynamic parameter client connection failed: {}".format(e))
                rospy.logwarn("Line following will run with default parameters")
            
            rospy.loginfo("Line following function started successfully")
            return True
            
        except Exception as e:
            rospy.logerr("Failed to start line following: {}".format(e))
            self.linefollow_active = False
            self.linefollow_process = None
            return False
    
    def apply_startup_params(self, cmd_data):
        """应用启动时的参数"""
        if not self.dyn_client:
            return
            
        try:
            params = {}
            
            # 从命令数据中提取参数
            if 'max_speed' in cmd_data:
                params['linear'] = float(cmd_data['max_speed'])
                
            if 'kp' in cmd_data:
                params['Kp'] = int(cmd_data['kp'])
            if 'ki' in cmd_data:
                params['Ki'] = int(cmd_data['ki']) 
            if 'kd' in cmd_data:
                params['Kd'] = int(cmd_data['kd'])
                
            # HSV参数
            if 'hsv_params' in cmd_data:
                hsv = cmd_data['hsv_params']
                if 'hmin' in hsv: params['Hmin'] = int(hsv['hmin'])
                if 'hmax' in hsv: params['Hmax'] = int(hsv['hmax'])
                if 'smin' in hsv: params['Smin'] = int(hsv['smin'])
                if 'smax' in hsv: params['Smax'] = int(hsv['smax'])
                if 'vmin' in hsv: params['Vmin'] = int(hsv['vmin'])
                if 'vmax' in hsv: params['Vmax'] = int(hsv['vmax'])
            
            if params:
                rospy.loginfo("Applying startup parameters: {}".format(params))
                self.dyn_client.update_configuration(params)
                rospy.loginfo("Startup parameters applied successfully")
                
        except Exception as e:
            rospy.logerr("Failed to apply startup parameters: {}".format(e))
    
    def stop_linefollow(self):
        """Stop line following function"""
        if self.linefollow_process is None:
            rospy.logwarn("Line following not running")
            return True
        
        try:
            rospy.loginfo("Stopping line following process...")
            
            # 首先尝试温和停止
            self.linefollow_process.terminate()
            
            # 等待进程结束
            wait_time = 0
            max_wait = 5
            while wait_time < max_wait:
                if self.linefollow_process.poll() is not None:
                    break
                rospy.sleep(0.5)
                wait_time += 0.5
            
            # 如果还没停止，强制杀死进程组
            if self.linefollow_process.poll() is None:
                rospy.logwarn("Process didn't stop gracefully, force killing...")
                try:
                    os.killpg(os.getpgid(self.linefollow_process.pid), signal.SIGTERM)
                    rospy.sleep(1.0)
                    if self.linefollow_process.poll() is None:
                        os.killpg(os.getpgid(self.linefollow_process.pid), signal.SIGKILL)
                except OSError as e:
                    rospy.logwarn("Error killing process group: {}".format(e))
                
            self.linefollow_process = None
            self.linefollow_active = False
            self.dyn_client = None
            
            rospy.loginfo("Line following function stopped successfully")
            return True
            
        except Exception as e:
            rospy.logerr("Failed to stop line following: {}".format(e))
            return False
    
    def reset_linefollow(self):
        """Reset line following function"""
        rospy.loginfo("Resetting line following function")
        if self.stop_linefollow():
            rospy.sleep(1.0)
            return self.start_linefollow(self.last_cmd_data)
        return False
    
    def set_linefollow_params(self, cmd_data):
        """Set line following parameters"""
        if self.dyn_client is None:
            rospy.logwarn("Dynamic parameter client not connected, cannot set parameters")
            return
        
        try:
            params = {}
            
            # PID parameters
            if 'kp' in cmd_data:
                params['Kp'] = cmd_data['kp']
            if 'ki' in cmd_data:
                params['Ki'] = cmd_data['ki']
            if 'kd' in cmd_data:
                params['Kd'] = cmd_data['kd']
            
            # Speed parameters
            if 'linear_speed' in cmd_data:
                params['linear'] = cmd_data['linear_speed']
            
            # HSV color range parameters
            if 'hsv_min' in cmd_data:
                hsv_min = cmd_data['hsv_min']
                params.update({
                    'Hmin': hsv_min[0],
                    'Smin': hsv_min[1],
                    'Vmin': hsv_min[2]
                })
            
            if 'hsv_max' in cmd_data:
                hsv_max = cmd_data['hsv_max']
                params.update({
                    'Hmax': hsv_max[0],
                    'Smax': hsv_max[1],
                    'Vmax': hsv_max[2]
                })
            
            if params:
                self.dyn_client.update_configuration(params)
                rospy.loginfo("Line follow parameters updated: {}".format(params))
            
        except Exception as e:
            rospy.logerr("Failed to set line follow parameters: {}".format(e))
    
    def cmd_vel_callback(self, msg):
        """Monitor cmd_vel topic for status feedback"""
        self.last_cmd_vel = msg
    
    def publish_status(self, event):
        """Publish line following status"""
        # Check process status
        process_running = False
        if self.linefollow_process is not None:
            process_running = self.linefollow_process.poll() is None
        
        # Detect if moving (based on cmd_vel)
        moving = (abs(self.last_cmd_vel.linear.x) > 0.01 or 
                 abs(self.last_cmd_vel.angular.z) > 0.01)
        
        status_data = {
            "linefollow_active": self.linefollow_active,  # 修改字段名匹配前端
            "process_running": process_running,
            "moving": moving,
            "linear_vel": self.last_cmd_vel.linear.x,
            "angular_vel": self.last_cmd_vel.angular.z,
            "timestamp": rospy.Time.now().to_sec()
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)
    
    def shutdown_hook(self):
        """Cleanup operations when node shuts down"""
        rospy.loginfo("Shutting down line follow controller...")
        self.stop_linefollow()

def main():
    try:
        linefollow_controller = LineFollowController()
        
        # Register shutdown hook
        rospy.on_shutdown(linefollow_controller.shutdown_hook)
        
        rospy.loginfo("Line follow controller ready")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Line follow controller interrupted by user")
    except Exception as e:
        rospy.logerr("Line follow controller runtime error: {}".format(e))

if __name__ == '__main__':
    main() 