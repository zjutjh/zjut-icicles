#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
import time
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from transbot_msgs.msg import *  # Import transbot custom messages
import json

class CompositeMainController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('composite_main_controller', anonymous=True)
        rospy.loginfo("=== TransBot Composite Application Main Controller Started ===")
        
        # System status
        self.current_mode = "manual"  # manual, linefollow, arm_demo
        self.system_active = True
        self.emergency_stop = False
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.status_pub = rospy.Publisher('/system_status', String, queue_size=1)
        
        # Subscribers
        self.web_cmd_sub = rospy.Subscriber('/web_cmd', String, self.web_cmd_callback)
        self.emergency_sub = rospy.Subscriber('/emergency_stop', Bool, self.emergency_callback)
        
        # Timer: publish system status periodically
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        
        rospy.loginfo("Main controller initialization completed")
        
    def web_cmd_callback(self, msg):
        """Handle control commands from Web interface"""
        try:
            cmd_data = json.loads(msg.data)
            cmd_type = cmd_data.get('cmd', '')
            value = cmd_data.get('value', 0.0)
            
            rospy.loginfo(f"Received Web command: {cmd_type}, value: {value}")
            
            # Handle emergency stop immediately (before checking emergency_stop status)
            if cmd_type == "emergency_stop":
                enable = cmd_data.get('enable', True)
                if isinstance(enable, dict):  # Handle nested enable format
                    enable = enable.get('enable', True)
                rospy.logwarn(f"Emergency stop command: enable={enable}")
                
                # Publish to emergency_stop topic
                emergency_msg = Bool()
                emergency_msg.data = enable
                emergency_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
                emergency_pub.publish(emergency_msg)
                return
            
            if self.emergency_stop:
                rospy.logwarn("Emergency stop active, ignoring command")
                return
                
            # Execute corresponding operation based on command type
            if cmd_type == "forward":
                self.move_robot(linear_x=value, angular_z=0.0)
            elif cmd_type == "backward":
                self.move_robot(linear_x=-value, angular_z=0.0)
            elif cmd_type == "left":
                self.move_robot(linear_x=0.0, angular_z=value)
            elif cmd_type == "right":
                self.move_robot(linear_x=0.0, angular_z=-value)
            elif cmd_type == "stop":
                self.move_robot(linear_x=0.0, angular_z=0.0)
            elif cmd_type == "mode_switch":
                # Handle both formats: direct mode or nested mode
                mode = cmd_data.get('mode')
                if isinstance(value, dict):  # Handle nested mode format
                    mode = value.get('mode', 'manual')
                elif mode is None:
                    mode = 'manual'
                self.switch_mode(mode)
            else:
                rospy.logwarn(f"Unknown command type: {cmd_type}")
                
        except json.JSONDecodeError:
            rospy.logerr(f"JSON parsing failed: {msg.data}")
        except Exception as e:
            rospy.logerr(f"Error handling Web command: {e}")
    
    def move_robot(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, 
                   angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """Send motion command to robot"""
        if self.emergency_stop:
            return
            
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.linear.z = linear_z
        twist.angular.x = angular_x
        twist.angular.y = angular_y
        twist.angular.z = angular_z
        
        self.cmd_vel_pub.publish(twist)
        rospy.logdebug(f"Motion command sent: linear=({linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}), "
                       f"angular=({angular_x:.2f}, {angular_y:.2f}, {angular_z:.2f})")
    
    def switch_mode(self, mode):
        """Switch system working mode"""
        valid_modes = ["manual", "linefollow", "arm_demo"]
        if mode in valid_modes:
            self.current_mode = mode
            rospy.loginfo(f"System mode switched to: {mode}")
        else:
            rospy.logwarn(f"Invalid mode: {mode}")
    
    def emergency_callback(self, msg):
        """Emergency stop callback"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            rospy.logwarn("=== Emergency Stop Activated ===")
            self.move_robot()  # Stop all motion
        else:
            rospy.loginfo("Emergency stop released")
    
    def publish_status(self, event):
        """Publish system status periodically"""
        status_data = {
            "timestamp": time.time(),
            "mode": self.current_mode,
            "active": self.system_active,
            "emergency_stop": self.emergency_stop,
            "node_name": "composite_main_controller"
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)
    
    def shutdown_hook(self):
        """Cleanup operations when node shuts down"""
        rospy.loginfo("Shutting down main controller...")
        self.system_active = False
        self.move_robot()  # Ensure robot stops
        rospy.loginfo("Main controller shutdown completed")

def main():
    try:
        controller = CompositeMainController()
        
        # Register shutdown hook
        rospy.on_shutdown(controller.shutdown_hook)
        
        rospy.loginfo("Main controller ready, waiting for commands...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted by user")
    except Exception as e:
        rospy.logerr(f"Main controller runtime error: {e}")

if __name__ == '__main__':
    main() 