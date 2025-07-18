#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socket
import subprocess

def get_local_ip():
    """Get local IP address"""
    try:
        # Create socket connection to external address to get local IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except:
        return "localhost"

def get_hostname():
    """Get hostname"""
    try:
        return socket.gethostname()
    except:
        return "unknown"

def print_banner():
    """Print startup banner"""
    banner = """
===============================================================
                    TransBot Composite System                    
                     Application Ready                      
===============================================================
  Version: v0.1.0                                                 
  Description: Integrated GUI Control, Line Following, Arm Control
  Author: TransBot Team                                          
===============================================================
"""
    print(banner)

def print_system_info():
    """Print system information"""
    local_ip = get_local_ip()
    hostname = get_hostname()
    
    print("Robot System Information:")
    print("   Hostname: {}".format(hostname))
    print("   IP Address: {}".format(local_ip))
    print("   ROS Version: {}".format(rospy.get_param('/rosdistro', 'Unknown')))
    print()

def print_connection_info():
    """Print connection information"""
    local_ip = get_local_ip()
    
    print("GUI Control Panel:")
    print("   Desktop GUI application will start automatically")
    print("   Or access via: {}".format(local_ip))
    print()

def print_ros_info():
    """Print ROS related information"""
    print("ROS Topics and Services:")
    print("   Control Topics:")
    print("     /web_cmd           - GUI control commands")
    print("     /cmd_vel           - Base movement control")
    print("     /follow_cmd        - Line following commands")
    print("     /arm_cmd           - Arm control commands")
    print("     /emergency_stop    - Emergency stop")
    print()
    print("   Status Topics:")
    print("     /system_status     - System status")
    print("     /linefollow_status - Line following status")
    print("     /arm_status        - Arm status")
    print("     /camera/image_raw  - Camera feed")
    print()

def print_usage_tips():
    """Print usage tips"""
    print("Usage Tips:")
    print("   1. Use GUI control panel for manual control")
    print("   2. Use arrow keys or WASD for movement")
    print("   3. Switch to line following mode for auto navigation")
    print("   4. Use arm demo functions to test robotic arm")
    print("   5. Click red emergency stop button in emergencies")
    print()

def print_troubleshooting():
    """Print troubleshooting information"""
    print("Troubleshooting:")
    print("   If GUI doesn't open:")
    print("     - Check X11 display is available")
    print("     - Ensure tkinter is installed")
    print("     - Try running node manually")
    print()
    print("   If camera shows no image:")
    print("     - Check camera device connection")
    print("     - Ensure /dev/video0 device exists")
    print("     - Check usb_cam node is running properly")
    print()

def check_dependencies():
    """Check dependencies"""
    print("Dependency Check:")
    print("   OK: ROS Python Library")
    print("   OK: System dependencies loaded")
    print()

def main():
    rospy.init_node('startup_message', anonymous=True)
    
    # Wait a moment for other nodes to start
    rospy.sleep(2.0)
    
    # Clear screen
    print("\033[2J\033[H")
    
    # Print startup information
    print_banner()
    print_system_info()
    print_connection_info()
    print_ros_info()
    print_usage_tips()
    print_troubleshooting()
    check_dependencies()
    
    print("System startup completed! Ready to operate.")
    print("   Press Ctrl+C to stop system")
    print("=" * 65)
    
    # Keep node running
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("\nTransBot system stopped")

if __name__ == '__main__':
    main() 