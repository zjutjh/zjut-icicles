#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import time
import sys
import math
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

# Import TransBot arm hardware library
sys.path.append("/root/Transbot/transbot")
try:
    from Transbot_Lib import Transbot
    from arm_transbot import Transbot_ARM
    HARDWARE_AVAILABLE = True
    rospy.loginfo("TransBot hardware library loaded successfully")
except ImportError as e:
    rospy.logwarn("TransBot hardware library not available: {}".format(e))
    HARDWARE_AVAILABLE = False

class ArmController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('arm_controller', anonymous=True)
        rospy.loginfo("=== Direct Motor Angle Arm Controller Started ===")
        
        # Hardware initialization
        self.hardware_available = HARDWARE_AVAILABLE
        self.bot = None
        self.bot_arm = None
        
        if self.hardware_available:
            try:
                self.bot_arm = Transbot_ARM()
                bot_arm_offset = self.bot_arm.get_arm_offset()
                self.bot = Transbot(arm_offset=bot_arm_offset)
                self.bot.create_receive_threading()
                rospy.loginfo("Hardware initialized successfully")
            except Exception as e:
                rospy.logerr("Hardware initialization failed: {}".format(e))
                self.hardware_available = False
        
        # Arm status
        self.arm_active = False
        self.current_motors = [150, 150, 150]  # [motor1, motor2, motor3] in degrees (0-300)
        self.target_motors = [150, 150, 150]
        
        # Motor limits (in degrees, 0-300 for servo motors)
        self.motor_limits = {
            'motor1': {'min': 0, 'max': 300},
            'motor2': {'min': 0, 'max': 300}, 
            'motor3': {'min': 0, 'max': 300}
        }
        
        # Predefined poses (direct motor angles in degrees)
        self.named_poses = {
            "init": [150, 150, 150],
            "up": [150, 180, 220],
            "down": [150, 250, 180],
            "home": [150, 150, 150],
            "ready": [150, 200, 200],
            "pick": [150, 230, 160],
            "place": [200, 200, 180]
        }
        
        # Movement parameters
        self.move_speed = 0.5  # Movement speed factor (0.1 - 1.0)
        self.interpolation_steps = 20  # Steps for smooth movement
        
        # Publishers
        self.status_pub = rospy.Publisher('/arm_status', String, queue_size=1)
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        
        # Subscribers
        self.arm_cmd_sub = rospy.Subscriber('/arm_cmd', String, self.arm_cmd_callback)
        self.arm_enable_sub = rospy.Subscriber('/arm_enable', Bool, self.enable_callback)
        
        # Timers
        self.status_timer = rospy.Timer(rospy.Duration(2.0), self.publish_status)
        self.joint_timer = rospy.Timer(rospy.Duration(0.5), self.publish_joint_states)
        
        rospy.loginfo("Direct motor angle arm controller initialization completed")
    
    def enable_callback(self, msg):
        """Enable/disable arm controller"""
        if msg.data:
            self.enable_arm()
        else:
            self.disable_arm()
    
    def enable_arm(self):
        """Enable arm control"""
        if not self.hardware_available:
            rospy.logwarn("Hardware not available, running in simulation mode")
        
        self.arm_active = True
        # Move to init position
        self.move_to_named_pose("init")
        rospy.loginfo("Arm controller enabled")
    
    def disable_arm(self):
        """Disable arm control"""
        self.arm_active = False
        rospy.loginfo("Arm controller disabled")
    
    def arm_cmd_callback(self, msg):
        """Process arm command"""
        try:
            cmd_data = json.loads(msg.data)
            cmd_type = cmd_data.get('cmd', '')
            
            rospy.loginfo("Received arm command: {}".format(cmd_type))
            
            # Enable and disable commands should always be processed
            if cmd_type == "enable":
                self.enable_arm()
                return
            elif cmd_type == "disable":
                self.disable_arm()
                return
            
            # Other commands require arm to be active
            if not self.arm_active:
                rospy.logwarn("Arm not active, ignoring command: {}".format(cmd_type))
                return
            
            if cmd_type == "move_motors":
                # Direct motor angle control
                motors = cmd_data.get('motors', [150, 150, 150])
                self.move_to_motors(motors)
            elif cmd_type == "move_pose":
                # Move to named pose
                pose_name = cmd_data.get('pose', 'init')
                self.move_to_named_pose(pose_name)
            elif cmd_type == "set_speed":
                # Set movement speed
                speed = cmd_data.get('speed', 0.5)
                self.set_move_speed(speed)
            elif cmd_type == "get_current_motors":
                self.get_current_motors()
            elif cmd_type == "stop":
                self.stop_movement()
            elif cmd_type == "home":
                self.move_to_named_pose("home")
            elif cmd_type == "demo":
                # Simple demo movement
                self.run_demo()
            else:
                rospy.logwarn("Unknown arm command: {}".format(cmd_type))
                
        except ValueError as e:
            rospy.logerr("Arm command JSON parsing failed: {}".format(msg.data))
        except Exception as e:
            rospy.logerr("Error processing arm command: {}".format(e))
    
    def move_to_motors(self, target_motors):
        """Move to specific motor angles (direct control)"""
        if len(target_motors) < 3:
            rospy.logerr("Invalid motor angles, need at least 3 values")
            return
        
        # Validate motor limits
        for i, angle in enumerate(target_motors[:3]):
            motor_name = 'motor{}'.format(i+1)
            if motor_name in self.motor_limits:
                limits = self.motor_limits[motor_name]
                if angle < limits['min'] or angle > limits['max']:
                    rospy.logwarn("Motor {} angle {} out of limits [{}, {}]".format(
                        i+1, angle, limits['min'], limits['max']))
                    return
        
        self.target_motors = target_motors[:3]
        rospy.loginfo("Moving to motor angles: {}".format(self.target_motors))
        
        # Perform smooth interpolated movement
        self.interpolated_move(self.current_motors, self.target_motors)
    
    def move_to_named_pose(self, pose_name):
        """Move to a predefined named pose"""
        if pose_name not in self.named_poses:
            rospy.logerr("Unknown pose name: {}".format(pose_name))
            return
        
        target_motors = self.named_poses[pose_name]
        rospy.loginfo("Moving to named pose '{}': {}".format(pose_name, target_motors))
        self.move_to_motors(target_motors)
    
    def interpolated_move(self, start_motors, end_motors):
        """Perform smooth interpolated movement between motor positions"""
        steps = self.interpolation_steps
        
        for step in range(steps + 1):
            ratio = float(step) / steps
            # Linear interpolation
            current_step = [
                start_motors[i] + (end_motors[i] - start_motors[i]) * ratio
                for i in range(3)
            ]
            
            # Send to hardware
            self.send_to_hardware(current_step)
            self.current_motors = current_step[:]
            
            # Sleep for smooth movement
            time.sleep(0.05 / self.move_speed)
        
        rospy.loginfo("Movement completed to: {}".format(end_motors))
    
    def send_to_hardware(self, motor_angles):
        """Send motor angles directly to hardware (no mapping)"""
        if not self.hardware_available or self.bot is None:
            return
        
        try:
            # Use motor angles directly without any mapping
            servo1 = motor_angles[0]
            servo2 = motor_angles[1]
            servo3 = motor_angles[2]
            
            # Clamp to valid servo range (0-300)
            servo1 = max(0, min(300, servo1))
            servo2 = max(0, min(300, servo2))
            servo3 = max(0, min(300, servo3))
            
            # Send to hardware
            self.bot.set_uart_servo_angle_array(servo1, servo2, servo3)
            
        except Exception as e:
            rospy.logerr("Failed to send commands to hardware: {}".format(e))
    
    def set_move_speed(self, speed):
        """Set movement speed factor"""
        self.move_speed = max(0.1, min(1.0, speed))
        rospy.loginfo("Movement speed set to: {}".format(self.move_speed))
    
    def get_current_motors(self):
        """Get current motor positions"""
        rospy.loginfo("Current motor angles: {}".format(self.current_motors))
        return self.current_motors
    
    def stop_movement(self):
        """Stop current movement"""
        # In a real implementation, this would halt any ongoing movement
        rospy.loginfo("Movement stopped")
    
    def run_demo(self):
        """Run a simple demonstration sequence"""
        rospy.loginfo("Starting arm demo sequence...")
        
        demo_sequence = [
            ("init", 2.0),
            ("ready", 2.0), 
            ("up", 2.0),
            ("down", 2.0),
            ("pick", 2.0),
            ("init", 2.0)
        ]
        
        for pose_name, wait_time in demo_sequence:
            if not self.arm_active:
                break
            self.move_to_named_pose(pose_name)
            rospy.sleep(wait_time)
        
        rospy.loginfo("Demo sequence completed")
    
    def publish_status(self, event):
        """Publish arm controller status"""
        status_data = {
            "arm_active": self.arm_active,
            "hardware_available": self.hardware_available,
            "current_motors": self.current_motors,
            "target_motors": self.target_motors,
            "move_speed": self.move_speed,
            "available_poses": list(self.named_poses.keys()),
            "motor_limits": self.motor_limits,
            "timestamp": rospy.Time.now().to_sec()
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)
    
    def publish_joint_states(self, event):
        """Publish current joint states"""
        if not self.arm_active:
            return
        
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['arm_motor1', 'arm_motor2', 'arm_motor3']
        # Convert degrees to radians for joint_states
        joint_state.position = [math.radians(angle) for angle in self.current_motors]
        joint_state.velocity = [0.0, 0.0, 0.0]
        joint_state.effort = [0.0, 0.0, 0.0]
        
        self.joint_state_pub.publish(joint_state)
    
    def shutdown_hook(self):
        """Cleanup when shutting down"""
        rospy.loginfo("Shutting down arm controller...")
        self.arm_active = False
        
        # Move to safe position
        if self.hardware_available:
            self.move_to_named_pose("init")

def main():
    try:
        arm_controller = ArmController()
        
        # Register shutdown hook
        rospy.on_shutdown(arm_controller.shutdown_hook)
        
        rospy.loginfo("Direct motor angle arm controller ready")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Arm controller interrupted by user")
    except Exception as e:
        rospy.logerr("Arm controller runtime error: {}".format(e))

if __name__ == '__main__':
    main() 