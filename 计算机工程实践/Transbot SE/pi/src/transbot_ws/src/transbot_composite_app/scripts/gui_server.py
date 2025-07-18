#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
import time
import json
import tkinter as tk
from tkinter import ttk, messagebox
import tkinter.font as tkFont
from std_msgs.msg import String, Bool

class TransBotGUI:
    
    # 巡线位置统一配置
    LINEFOLLOW_PITCH = 40    # 巡线俯仰角度
    LINEFOLLOW_YAW = 92      # 巡线水平角度
    
    def __init__(self, root):
        self.root = root
        self.root.title("TransBot Control Panel")
        self.root.geometry("1000x600")  # 降低高度
        self.root.configure(bg='#2c3e50')
        
        # Initialization flag
        self.initialization_complete = False
        
        # Initialize ROS node
        rospy.init_node('transbot_gui', anonymous=True)
        rospy.loginfo("=== TransBot GUI Started ===")
        
        # Robot status
        self.robot_status = {
            "mode": "manual",
            "moving": False,
            "linefollow_active": False,
            "arm_active": False,
            "emergency_stop": False
        }
        
        # Camera status
        self.camera_status = {
            "current_angle": 60,
            "target_angle": 20,
            "is_moving": False,
            "min_angle": 0,
            "max_angle": 120
        }
        
        # ROS Publishers
        self.web_cmd_pub = rospy.Publisher('/web_cmd', String, queue_size=10)
        self.linefollow_control_pub = rospy.Publisher('/linefollow_control', String, queue_size=10)
        self.camera_cmd_pub = rospy.Publisher('/camera_cmd', String, queue_size=10)
        self.arm_cmd_pub = rospy.Publisher('/arm_cmd', String, queue_size=10)
        self.emergency_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
        
        # ROS Subscribers
        self.status_sub = rospy.Subscriber('/system_status', String, self.status_callback)
        self.linefollow_status_sub = rospy.Subscriber('/linefollow_status', String, self.linefollow_status_callback)
        self.camera_status_sub = rospy.Subscriber('/camera_status', String, self.camera_status_callback)
        self.arm_status_sub = rospy.Subscriber('/arm_status', String, self.arm_status_callback)
        
        # Setup GUI
        self.setup_gui()
        
        # Start ROS spinning in separate thread
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Mark initialization as complete
        self.initialization_complete = True
        
        rospy.loginfo("TransBot GUI initialization completed")
    
    def setup_gui(self):
        """Setup the main GUI layout"""
        # Main title
        title_frame = tk.Frame(self.root, bg='#34495e', height=50)
        title_frame.pack(fill='x', padx=10, pady=5)
        title_frame.pack_propagate(False)
        
        title_label = tk.Label(title_frame, text="TransBot Control Panel", 
                              font=('Arial', 20, 'bold'), fg='white', bg='#34495e')
        title_label.pack(expand=True)
        
        # Main content area
        main_frame = tk.Frame(self.root, bg='#2c3e50')
        main_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Left panel - Status
        left_panel = tk.Frame(main_frame, bg='#34495e', width=300)
        left_panel.pack(side='left', fill='y', padx=(0, 5))
        left_panel.pack_propagate(False)
        
        # Status display
        status_frame = tk.LabelFrame(left_panel, text="Robot Status", 
                                    font=('Arial', 12, 'bold'), fg='white', bg='#34495e')
        status_frame.pack(fill='x', padx=10, pady=10)
        
        self.setup_status_display(status_frame)
        
        # Right panel - Controls (wider now)
        right_panel = tk.Frame(main_frame, bg='#34495e')
        right_panel.pack(side='right', fill='both', expand=True, padx=(5, 0))
        
        # Create notebook for tabbed controls
        self.notebook = ttk.Notebook(right_panel)
        self.notebook.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Configure notebook style
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TNotebook', background='#34495e')
        style.configure('TNotebook.Tab', background='#2c3e50', foreground='white', 
                       padding=[20, 10])
        style.map('TNotebook.Tab', background=[('selected', '#3498db')])
        
        self.setup_control_tabs()
    
    def setup_status_display(self, parent):
        """Setup status display widgets"""
        # Status variables
        self.status_vars = {
            "mode": tk.StringVar(value="manual"),
            "moving": tk.StringVar(value="Stopped"),
            "linefollow": tk.StringVar(value="Inactive"),
            "arm": tk.StringVar(value="Inactive"),
            "emergency": tk.StringVar(value="Normal")
        }
        
        # Status grid
        row = 0
        for key, var in self.status_vars.items():
            label = tk.Label(parent, text=f"{key.replace('_', ' ').title()}:", 
                           font=('Arial', 10, 'bold'), fg='white', bg='#34495e')
            label.grid(row=row, column=0, sticky='w', padx=10, pady=2)
            
            value_label = tk.Label(parent, textvariable=var, 
                                 font=('Arial', 10), fg='#3498db', bg='#34495e')
            value_label.grid(row=row, column=1, sticky='w', padx=10, pady=2)
            row += 1
    
    def setup_control_tabs(self):
        """Setup control tabs"""
        # Manual Control Tab
        manual_frame = tk.Frame(self.notebook, bg='#34495e')
        self.notebook.add(manual_frame, text='手动控制')
        self.setup_manual_controls(manual_frame)
        
        # Camera Control Tab
        camera_frame = tk.Frame(self.notebook, bg='#34495e')
        self.notebook.add(camera_frame, text='摄像头控制')
        self.setup_camera_controls(camera_frame)
        
        # Line Following Tab
        linefollow_frame = tk.Frame(self.notebook, bg='#34495e')
        self.notebook.add(linefollow_frame, text='巡线控制')
        self.setup_linefollow_controls(linefollow_frame)
        
        # Arm Control Tab
        arm_frame = tk.Frame(self.notebook, bg='#34495e')
        self.notebook.add(arm_frame, text='机械臂控制')
        self.setup_arm_controls(arm_frame)
        
        # Emergency Tab
        emergency_frame = tk.Frame(self.notebook, bg='#34495e')
        self.notebook.add(emergency_frame, text='紧急停止')
        self.setup_emergency_controls(emergency_frame)
    
    def setup_manual_controls(self, parent):
        """Setup manual control buttons"""
        # Movement controls
        move_frame = tk.LabelFrame(parent, text="移动控制", 
                                  font=('Arial', 12, 'bold'), fg='white', bg='#34495e')
        move_frame.pack(fill='x', padx=20, pady=20)
        
        # Movement buttons grid
        button_grid = tk.Frame(move_frame, bg='#34495e')
        button_grid.pack(pady=20)
        
        # Forward
        forward_btn = tk.Button(button_grid, text="↑", font=('Arial', 20, 'bold'),
                               width=4, height=2, bg='#3498db', fg='white',
                               command=lambda: self.send_move_command("forward"))
        forward_btn.grid(row=0, column=1, padx=5, pady=5)
        
        # Left, Stop, Right
        left_btn = tk.Button(button_grid, text="←", font=('Arial', 20, 'bold'),
                            width=4, height=2, bg='#3498db', fg='white',
                            command=lambda: self.send_move_command("left"))
        left_btn.grid(row=1, column=0, padx=5, pady=5)
        
        stop_btn = tk.Button(button_grid, text="⊠", font=('Arial', 20, 'bold'),
                            width=4, height=2, bg='#e74c3c', fg='white',
                            command=lambda: self.send_move_command("stop"))
        stop_btn.grid(row=1, column=1, padx=5, pady=5)
        
        right_btn = tk.Button(button_grid, text="→", font=('Arial', 20, 'bold'),
                             width=4, height=2, bg='#3498db', fg='white',
                             command=lambda: self.send_move_command("right"))
        right_btn.grid(row=1, column=2, padx=5, pady=5)
        
        # Backward
        backward_btn = tk.Button(button_grid, text="↓", font=('Arial', 20, 'bold'),
                                width=4, height=2, bg='#3498db', fg='white',
                                command=lambda: self.send_move_command("backward"))
        backward_btn.grid(row=2, column=1, padx=5, pady=5)
        
        # Speed control
        speed_frame = tk.LabelFrame(parent, text="速度控制", 
                                   font=('Arial', 12, 'bold'), fg='white', bg='#34495e')
        speed_frame.pack(fill='x', padx=20, pady=20)
        
        speed_control = tk.Frame(speed_frame, bg='#34495e')
        speed_control.pack(pady=15)
        
        tk.Label(speed_control, text="速度:", font=('Arial', 12, 'bold'), 
                fg='white', bg='#34495e').pack(side='left')
        
        self.speed_var = tk.DoubleVar(value=0.5)
        speed_scale = tk.Scale(speed_control, from_=0.1, to=1.0, resolution=0.1,
                              orient='horizontal', variable=self.speed_var,
                              bg='#34495e', fg='white', highlightbackground='#34495e',
                              length=300)
        speed_scale.pack(side='right', padx=20)
    
    def setup_camera_controls(self, parent):
        """Setup camera control widgets"""
        # Pitch control
        pitch_frame = tk.LabelFrame(parent, text="俯仰角度控制 (上下: 10-140度)", 
                                   font=('Arial', 12, 'bold'), fg='white', bg='#34495e')
        pitch_frame.pack(fill='x', padx=20, pady=10)
        
        pitch_display = tk.Frame(pitch_frame, bg='#34495e')
        pitch_display.pack(pady=10)
        
        tk.Label(pitch_display, text="当前角度:", font=('Arial', 11), 
                fg='white', bg='#34495e').pack(side='left')
        
        self.camera_pitch_var = tk.StringVar(value=f"{self.LINEFOLLOW_PITCH}度")
        pitch_label = tk.Label(pitch_display, textvariable=self.camera_pitch_var, 
                              font=('Arial', 11, 'bold'), fg='#3498db', bg='#34495e')
        pitch_label.pack(side='left', padx=(10, 0))
        
        # 数字输入控制
        pitch_input_frame = tk.Frame(pitch_frame, bg='#34495e')
        pitch_input_frame.pack(pady=10)
        
        tk.Label(pitch_input_frame, text="设置角度:", font=('Arial', 11), 
                fg='white', bg='#34495e').pack(side='left')
        
        self.pitch_entry = tk.Entry(pitch_input_frame, font=('Arial', 11), width=6)
        self.pitch_entry.pack(side='left', padx=(10, 5))
        self.pitch_entry.insert(0, str(self.LINEFOLLOW_PITCH))
        
        tk.Label(pitch_input_frame, text="度", font=('Arial', 11), 
                fg='white', bg='#34495e').pack(side='left', padx=(0, 10))
        
        pitch_set_btn = tk.Button(pitch_input_frame, text="设置", 
                                 font=('Arial', 10, 'bold'),
                                 bg='#3498db', fg='white', 
                                 command=self.set_pitch_from_entry,
                                 width=6, height=1)
        pitch_set_btn.pack(side='left', padx=5)
        
        # Yaw control
        yaw_frame = tk.LabelFrame(parent, text="水平角度控制 (左右: 0-180度)", 
                                 font=('Arial', 12, 'bold'), fg='white', bg='#34495e')
        yaw_frame.pack(fill='x', padx=20, pady=10)
        
        yaw_display = tk.Frame(yaw_frame, bg='#34495e')
        yaw_display.pack(pady=10)
        
        tk.Label(yaw_display, text="当前角度:", font=('Arial', 11), 
                fg='white', bg='#34495e').pack(side='left')
        
        self.camera_yaw_var = tk.StringVar(value=f"{self.LINEFOLLOW_YAW}度")
        yaw_label = tk.Label(yaw_display, textvariable=self.camera_yaw_var, 
                            font=('Arial', 11, 'bold'), fg='#3498db', bg='#34495e')
        yaw_label.pack(side='left', padx=(10, 0))
        
        # 数字输入控制
        yaw_input_frame = tk.Frame(yaw_frame, bg='#34495e')
        yaw_input_frame.pack(pady=10)
        
        tk.Label(yaw_input_frame, text="设置角度:", font=('Arial', 11), 
                fg='white', bg='#34495e').pack(side='left')
        
        self.yaw_entry = tk.Entry(yaw_input_frame, font=('Arial', 11), width=6)
        self.yaw_entry.pack(side='left', padx=(10, 5))
        self.yaw_entry.insert(0, str(self.LINEFOLLOW_YAW))
        
        tk.Label(yaw_input_frame, text="度", font=('Arial', 11), 
                fg='white', bg='#34495e').pack(side='left', padx=(0, 10))
        
        yaw_set_btn = tk.Button(yaw_input_frame, text="设置", 
                               font=('Arial', 10, 'bold'),
                               bg='#3498db', fg='white', 
                               command=self.set_yaw_from_entry,
                               width=6, height=1)
        yaw_set_btn.pack(side='left', padx=5)
        
        # Quick control buttons
        button_frame = tk.LabelFrame(parent, text="快速控制", 
                                    font=('Arial', 12, 'bold'), fg='white', bg='#34495e')
        button_frame.pack(fill='x', padx=20, pady=10)
        
        buttons = tk.Frame(button_frame, bg='#34495e')
        buttons.pack(pady=15)
        
        linefollow_btn = tk.Button(buttons, text="巡线位置", 
                                  font=('Arial', 11, 'bold'),
                                  bg='#27ae60', fg='white', 
                                  command=self.set_linefollow_position,
                                  width=12, height=2)
        linefollow_btn.pack(side='left', padx=10)
        
        center_btn = tk.Button(buttons, text="中心位置", 
                              font=('Arial', 11, 'bold'),
                              bg='#3498db', fg='white', 
                              command=self.set_camera_center,
                              width=12, height=2)
        center_btn.pack(side='left', padx=10)
        
        # Direction control
        direction_frame = tk.LabelFrame(parent, text="方向微调", 
                                       font=('Arial', 12, 'bold'), fg='white', bg='#34495e')
        direction_frame.pack(fill='x', padx=20, pady=10)
        
        direction_grid = tk.Frame(direction_frame, bg='#34495e')
        direction_grid.pack(pady=15)
        
        up_btn = tk.Button(direction_grid, text="↑", 
                          font=('Arial', 16, 'bold'),
                          bg='#95a5a6', fg='white', 
                          command=self.camera_up,
                          width=4, height=2)
        up_btn.grid(row=0, column=1, padx=5, pady=5)
        
        left_btn = tk.Button(direction_grid, text="←", 
                            font=('Arial', 16, 'bold'),
                            bg='#95a5a6', fg='white', 
                            command=self.camera_left,
                            width=4, height=2)
        left_btn.grid(row=1, column=0, padx=5, pady=5)
        
        center_control_btn = tk.Button(direction_grid, text="●", 
                                      font=('Arial', 16, 'bold'),
                                      bg='#34495e', fg='white', 
                                      command=self.set_camera_center,
                                      width=4, height=2)
        center_control_btn.grid(row=1, column=1, padx=5, pady=5)
        
        right_btn = tk.Button(direction_grid, text="→", 
                             font=('Arial', 16, 'bold'),
                             bg='#95a5a6', fg='white', 
                             command=self.camera_right,
                             width=4, height=2)
        right_btn.grid(row=1, column=2, padx=5, pady=5)
        
        down_btn = tk.Button(direction_grid, text="↓", 
                            font=('Arial', 16, 'bold'),
                            bg='#95a5a6', fg='white', 
                            command=self.camera_down,
                            width=4, height=2)
        down_btn.grid(row=2, column=1, padx=5, pady=5)
    
    def setup_linefollow_controls(self, parent):
        """Setup line following control buttons"""
        # Status display
        status_frame = tk.LabelFrame(parent, text="巡线状态", 
                                    font=('Arial', 12, 'bold'), fg='white', bg='#34495e')
        status_frame.pack(fill='x', padx=20, pady=10)
        
        status_display = tk.Frame(status_frame, bg='#34495e')
        status_display.pack(pady=15)
        
        tk.Label(status_display, text="当前状态:", font=('Arial', 11), 
                fg='white', bg='#34495e').pack(side='left')
        
        self.linefollow_status_var = tk.StringVar(value="未激活")
        status_label = tk.Label(status_display, textvariable=self.linefollow_status_var, 
                               font=('Arial', 11, 'bold'), fg='#e74c3c', bg='#34495e')
        status_label.pack(side='left', padx=(10, 0))
        
        # Speed control
        speed_frame = tk.LabelFrame(parent, text="速度控制", 
                                   font=('Arial', 12, 'bold'), fg='white', bg='#34495e')
        speed_frame.pack(fill='x', padx=20, pady=10)
        
        speed_control = tk.Frame(speed_frame, bg='#34495e')
        speed_control.pack(pady=15)
        
        tk.Label(speed_control, text="巡线速度:", font=('Arial', 11), 
                fg='white', bg='#34495e').pack(side='left')
        
        self.linefollow_speed = tk.DoubleVar(value=0.3)
        speed_scale = tk.Scale(speed_control, from_=0.1, to=1.0, resolution=0.1,
                              orient='horizontal', variable=self.linefollow_speed,
                              bg='#34495e', fg='white', highlightthickness=0,
                              length=300)
        speed_scale.pack(side='right', padx=20)
        
        # Control buttons
        button_frame = tk.LabelFrame(parent, text="巡线控制", 
                                    font=('Arial', 12, 'bold'), fg='white', bg='#34495e')
        button_frame.pack(fill='x', padx=20, pady=10)
        
        buttons = tk.Frame(button_frame, bg='#34495e')
        buttons.pack(pady=20)
        
        start_btn = tk.Button(buttons, text="开始巡线", 
                             font=('Arial', 14, 'bold'),
                             bg='#27ae60', fg='white', 
                             command=self.start_linefollow,
                             width=12, height=3)
        start_btn.pack(side='left', padx=10)
        
        stop_btn = tk.Button(buttons, text="停止巡线", 
                            font=('Arial', 14, 'bold'),
                            bg='#e74c3c', fg='white', 
                            command=self.stop_linefollow,
                            width=12, height=3)
        stop_btn.pack(side='left', padx=10)
        
        # 添加更新速度按钮
        update_speed_btn = tk.Button(buttons, text="更新速度", 
                                    font=('Arial', 12, 'bold'),
                                    bg='#f39c12', fg='white', 
                                    command=self.update_linefollow_speed,
                                    width=10, height=2)
        update_speed_btn.pack(side='left', padx=10)
    
    def setup_arm_controls(self, parent):
        """Setup arm control buttons"""
        # Status display
        status_frame = tk.LabelFrame(parent, text="机械臂状态", 
                                    font=('Arial', 12, 'bold'), fg='white', bg='#34495e')
        status_frame.pack(fill='x', padx=20, pady=10)
        
        status_display = tk.Frame(status_frame, bg='#34495e')
        status_display.pack(pady=10)
        
        tk.Label(status_display, text="当前状态:", font=('Arial', 11), 
                fg='white', bg='#34495e').pack(side='left')
        
        self.arm_status_var = tk.StringVar(value="未激活")
        status_label = tk.Label(status_display, textvariable=self.arm_status_var, 
                               font=('Arial', 11, 'bold'), fg='#e74c3c', bg='#34495e')
        status_label.pack(side='left', padx=(10, 0))
        
        # Current motor angles display
        angles_frame = tk.LabelFrame(parent, text="当前电机角度 (度)", 
                                    font=('Arial', 12, 'bold'), fg='white', bg='#34495e')
        angles_frame.pack(fill='x', padx=20, pady=10)
        
        angles_display = tk.Frame(angles_frame, bg='#34495e')
        angles_display.pack(pady=10)
        
        # Motor angle displays
        self.motor_angle_vars = [tk.StringVar(value="150"), tk.StringVar(value="150"), tk.StringVar(value="150")]
        
        for i in range(3):
            motor_frame = tk.Frame(angles_display, bg='#34495e')
            motor_frame.pack(side='left', padx=20)
            
            tk.Label(motor_frame, text="电机{}:".format(i+1), font=('Arial', 10), 
                    fg='white', bg='#34495e').pack()
            
            angle_label = tk.Label(motor_frame, textvariable=self.motor_angle_vars[i], 
                                  font=('Arial', 12, 'bold'), fg='#3498db', bg='#34495e')
            angle_label.pack()
        
        # Direct motor control
        control_frame = tk.LabelFrame(parent, text="直接电机角度控制 (0-300度)", 
                                     font=('Arial', 12, 'bold'), fg='white', bg='#34495e')
        control_frame.pack(fill='x', padx=20, pady=10)
        
        # Motor input fields
        input_frame = tk.Frame(control_frame, bg='#34495e')
        input_frame.pack(pady=15)
        
        self.motor_entries = []
        
        for i in range(3):
            motor_input_frame = tk.Frame(input_frame, bg='#34495e')
            motor_input_frame.pack(side='left', padx=15)
            
            tk.Label(motor_input_frame, text="电机{}:".format(i+1), font=('Arial', 10), 
                    fg='white', bg='#34495e').pack()
            
            entry = tk.Entry(motor_input_frame, font=('Arial', 11), width=6)
            entry.pack(pady=5)
            entry.insert(0, "150")  # Default angle
            self.motor_entries.append(entry)
            
            tk.Label(motor_input_frame, text="度", font=('Arial', 10), 
                    fg='white', bg='#34495e').pack()
        
        # Control buttons
        button_frame = tk.Frame(control_frame, bg='#34495e')
        button_frame.pack(pady=15)
        
        move_btn = tk.Button(button_frame, text="移动到指定角度", 
                            font=('Arial', 12, 'bold'),
                            bg='#3498db', fg='white', 
                            command=self.move_to_motor_angles,
                            width=15, height=2)
        move_btn.pack(side='left', padx=10)
        
        get_current_btn = tk.Button(button_frame, text="获取当前角度", 
                                   font=('Arial', 12, 'bold'),
                                   bg='#95a5a6', fg='white', 
                                   command=self.get_current_motor_angles,
                                   width=15, height=2)
        get_current_btn.pack(side='left', padx=10)
        
        # Enable/Disable controls
        enable_frame = tk.LabelFrame(parent, text="机械臂开关", 
                                    font=('Arial', 12, 'bold'), fg='white', bg='#34495e')
        enable_frame.pack(fill='x', padx=20, pady=10)
        
        enable_buttons = tk.Frame(enable_frame, bg='#34495e')
        enable_buttons.pack(pady=15)
        
        enable_btn = tk.Button(enable_buttons, text="启用机械臂", 
                              font=('Arial', 12, 'bold'), bg='#27ae60', fg='white',
                              command=self.enable_arm, width=12, height=2)
        enable_btn.pack(side='left', padx=10)
        
        disable_btn = tk.Button(enable_buttons, text="禁用机械臂", 
                               font=('Arial', 12, 'bold'), bg='#e74c3c', fg='white',
                               command=self.disable_arm, width=12, height=2)
        disable_btn.pack(side='left', padx=10)
        
        # Preset positions (using direct motor angles)
        preset_frame = tk.LabelFrame(parent, text="预设位置", 
                                    font=('Arial', 12, 'bold'), fg='white', bg='#34495e')
        preset_frame.pack(fill='x', padx=20, pady=10)
        
        preset_buttons = tk.Frame(preset_frame, bg='#34495e')
        preset_buttons.pack(pady=10)
        
        # First row of preset buttons
        preset_row1 = tk.Frame(preset_buttons, bg='#34495e')
        preset_row1.pack(pady=5)
        
        init_btn = tk.Button(preset_row1, text="初始位置", 
                            font=('Arial', 10, 'bold'), bg='#95a5a6', fg='white',
                            command=lambda: self.send_arm_pose_command("init"),
                            width=10, height=2)
        init_btn.pack(side='left', padx=5)
        
        home_btn = tk.Button(preset_row1, text="归位", 
                            font=('Arial', 10, 'bold'), bg='#95a5a6', fg='white',
                            command=lambda: self.send_arm_pose_command("home"),
                            width=10, height=2)
        home_btn.pack(side='left', padx=5)
        
        up_btn = tk.Button(preset_row1, text="抬起", 
                          font=('Arial', 10, 'bold'), bg='#95a5a6', fg='white',
                          command=lambda: self.send_arm_pose_command("up"),
                          width=10, height=2)
        up_btn.pack(side='left', padx=5)
        
        down_btn = tk.Button(preset_row1, text="放下", 
                            font=('Arial', 10, 'bold'), bg='#95a5a6', fg='white',
                            command=lambda: self.send_arm_pose_command("down"),
                            width=10, height=2)
        down_btn.pack(side='left', padx=5)
        
        # Second row of preset buttons
        preset_row2 = tk.Frame(preset_buttons, bg='#34495e')
        preset_row2.pack(pady=5)
        
        ready_btn = tk.Button(preset_row2, text="准备", 
                             font=('Arial', 10, 'bold'), bg='#95a5a6', fg='white',
                             command=lambda: self.send_arm_pose_command("ready"),
                             width=10, height=2)
        ready_btn.pack(side='left', padx=5)
        
        pick_btn = tk.Button(preset_row2, text="抓取", 
                            font=('Arial', 10, 'bold'), bg='#95a5a6', fg='white',
                            command=lambda: self.send_arm_pose_command("pick"),
                            width=10, height=2)
        pick_btn.pack(side='left', padx=5)
        
        place_btn = tk.Button(preset_row2, text="放置", 
                             font=('Arial', 10, 'bold'), bg='#95a5a6', fg='white',
                             command=lambda: self.send_arm_pose_command("place"),
                             width=10, height=2)
        place_btn.pack(side='left', padx=5)
        
        demo_btn = tk.Button(preset_row2, text="演示", 
                            font=('Arial', 10, 'bold'), bg='#f39c12', fg='white',
                            command=self.start_arm_demo,
                            width=10, height=2)
        demo_btn.pack(side='left', padx=5)
    
    def send_move_command(self, direction):
        """Send movement command"""
        try:
            cmd_data = {
                "cmd": direction,
                "value": self.speed_var.get()
            }
            
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.web_cmd_pub.publish(msg)
            
            rospy.loginfo(f"Move command sent: {direction} at speed {self.speed_var.get()}")
            
        except Exception as e:
            rospy.logerr(f"Move command error: {e}")
            messagebox.showerror("Error", f"Failed to send move command: {e}")
    
    def start_linefollow(self):
        """Start line following"""
        try:
            rospy.loginfo("=== Starting line follow from GUI ===")
            
            # 首先设置默认的HSV参数
            self.set_default_hsv_params()
            
            # 获取GUI中设置的巡线速度
            linefollow_speed = self.linefollow_speed.get()
            rospy.loginfo("Using linefollow speed: {}".format(linefollow_speed))
            
            # 发送启动巡线命令到新的控制器，包含速度参数
            cmd_data = {
                "cmd": "start",
                "speed": linefollow_speed
            }
            
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.linefollow_control_pub.publish(msg)
            
            rospy.loginfo("Line follow start command sent successfully with speed: {}".format(linefollow_speed))
            
        except Exception as e:
            rospy.logerr("Line follow start error: {}".format(e))
            self.show_error_dialog("启动错误", str(e))
    
    def set_default_hsv_params(self):
        """设置默认的HSV参数到transbot_linefollow包"""
        try:
            import os
            import shutil
            
            # 使用固定的工作空间路径
            workspace_root = "/root/transbot_ws"
            
            # 默认HSV参数文件路径
            default_hsv_file = os.path.join(workspace_root, "src/transbot_composite_app/config/default_linefollow_hsv.txt")
            # transbot_linefollow的HSV参数文件路径
            target_hsv_file = os.path.join(workspace_root, "src/transbot_linefollow/scripts/LineFollowHSV.text")
            
            rospy.loginfo("Using workspace root: {}".format(workspace_root))
            rospy.loginfo("Default HSV file: {}".format(default_hsv_file))
            rospy.loginfo("Target HSV file: {}".format(target_hsv_file))
            
            # 如果默认参数文件存在，复制到目标位置
            if os.path.exists(default_hsv_file):
                # 确保目标目录存在
                target_dir = os.path.dirname(target_hsv_file)
                if not os.path.exists(target_dir):
                    os.makedirs(target_dir)
                
                shutil.copy2(default_hsv_file, target_hsv_file)
                rospy.loginfo("Default HSV parameters copied to transbot_linefollow")
                
                # 读取并显示参数
                with open(default_hsv_file, 'r') as f:
                    hsv_content = f.read().strip()
                    rospy.loginfo("Using HSV parameters: {}".format(hsv_content))
            else:
                rospy.logwarn("Default HSV parameter file not found: {}".format(default_hsv_file))
                
        except Exception as e:
            rospy.logerr("Failed to set default HSV parameters: {}".format(e))
    
    def show_error_dialog(self, title, message):
        """在主线程中安全显示错误对话框"""
        try:
            messagebox.showerror(title, message)
        except Exception as e:
            rospy.logerr("Failed to show error dialog: {}".format(e))
    
    def load_linefollow_params(self):
        """Load line following parameters from config file or parameter server"""
        try:
            params = {}
            
            # 尝试从参数服务器读取linefollow参数
            if rospy.has_param('/linefollow_controller'):
                linefollow_ns = rospy.get_param('/linefollow_controller', {})
                
                # PID参数
                if 'pid' in linefollow_ns:
                    pid_params = linefollow_ns['pid']
                    params['kp'] = pid_params.get('kp', 40)
                    params['ki'] = pid_params.get('ki', 0) 
                    params['kd'] = pid_params.get('kd', 12)
                
                # 速度参数
                if 'control' in linefollow_ns:
                    control_params = linefollow_ns['control']
                    if 'max_linear_speed' in control_params:
                        params['max_speed'] = control_params['max_linear_speed']
                
                # HSV参数
                if 'image_processing' in linefollow_ns and 'line_color' in linefollow_ns['image_processing']:
                    color_params = linefollow_ns['image_processing']['line_color']
                    if 'lower_hsv' in color_params and 'upper_hsv' in color_params:
                        params['hsv_params'] = {
                            'hmin': color_params['lower_hsv'][0],
                            'smin': color_params['lower_hsv'][1],
                            'vmin': color_params['lower_hsv'][2],
                            'hmax': color_params['upper_hsv'][0],
                            'smax': color_params['upper_hsv'][1],
                            'vmax': color_params['upper_hsv'][2]
                        }
            
            # 尝试读取transbot_linefollow的保存参数
            if not params.get('hsv_params'):
                hsv_from_file = self.read_transbot_linefollow_hsv()
                if hsv_from_file:
                    params['hsv_params'] = hsv_from_file
                    rospy.loginfo("Using HSV parameters from transbot_linefollow package")
            
            # 如果参数服务器没有参数，使用transbot_linefollow兼容的默认值
            if not params:
                params = {
                    'kp': 40,  # 使用transbot_linefollow的默认PID
                    'ki': 0,
                    'kd': 12,
                    'max_speed': 0.3,
                    'hsv_params': {
                        'hmin': 0,
                        'smin': 85,
                        'vmin': 126,
                        'hmax': 9,
                        'smax': 253,
                        'vmax': 255
                    }
                }
                rospy.loginfo("Using transbot_linefollow compatible default parameters")
            else:
                rospy.loginfo("Loaded linefollow parameters from config")
                
            return params
            
        except Exception as e:
            rospy.logerr("Failed to load linefollow parameters: {}".format(e))
            # 返回transbot_linefollow兼容的默认参数
            return {
                'kp': 40,
                'ki': 0,
                'kd': 12,
                'max_speed': 0.3
            }
    
    def read_transbot_linefollow_hsv(self):
        """读取transbot_linefollow包保存的HSV参数"""
        try:
            import os
            
            # 使用固定的工作空间路径
            workspace_root = "/root/transbot_ws"
            
            hsv_file = os.path.join(workspace_root, "src/transbot_linefollow/scripts/LineFollowHSV.text")
            rospy.loginfo("Reading HSV from: {}".format(hsv_file))
            
            if os.path.exists(hsv_file):
                with open(hsv_file, 'r') as f:
                    line = f.readline().strip()
                    if line:
                        values = [int(x.strip()) for x in line.split(',')]
                        if len(values) == 6:
                            rospy.loginfo("Successfully read HSV parameters: {}".format(values))
                            return {
                                'hmin': values[0],
                                'smin': values[1], 
                                'vmin': values[2],
                                'hmax': values[3],
                                'smax': values[4],
                                'vmax': values[5]
                            }
            else:
                rospy.logwarn("HSV file not found: {}".format(hsv_file))
            return None
        except Exception as e:
            rospy.logwarn("Failed to read transbot_linefollow HSV file: {}".format(e))
            return None
    
    def stop_linefollow(self):
        """Stop line following"""
        try:
            rospy.loginfo("=== Stopping line follow from GUI ===")
            
            # 发送停止巡线命令
            cmd_data = {
                "cmd": "stop"
            }
            
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.linefollow_control_pub.publish(msg)
            
            rospy.loginfo("Line follow stop command sent successfully")
            
        except Exception as e:
            rospy.logerr("Line follow stop error: {}".format(e))
            self.show_error_dialog("停止错误", str(e))
    
    def enable_arm(self):
        """Enable arm controller"""
        try:
            cmd_data = {"cmd": "enable"}
            
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.arm_cmd_pub.publish(msg)
            
            rospy.loginfo("Arm enable command sent")
            
        except Exception as e:
            rospy.logerr(f"Arm enable error: {e}")
            messagebox.showerror("Error", f"Failed to enable arm: {e}")
    
    def disable_arm(self):
        """Disable arm controller"""
        try:
            cmd_data = {"cmd": "disable"}
            
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.arm_cmd_pub.publish(msg)
            
            rospy.loginfo("Arm disable command sent")
            
        except Exception as e:
            rospy.logerr(f"Arm disable error: {e}")
            messagebox.showerror("Error", f"Failed to disable arm: {e}")

    def start_arm_demo(self):
        """Start arm demo"""
        try:
            cmd_data = {"cmd": "demo"}
            
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.arm_cmd_pub.publish(msg)
            
            rospy.loginfo("Arm demo command sent")
            
        except Exception as e:
            rospy.logerr(f"Arm demo start error: {e}")
            messagebox.showerror("Error", f"Failed to start arm demo: {e}")

    def send_arm_pose_command(self, pose_name):
        """Send arm preset pose command"""
        try:
            cmd_data = {
                "cmd": "move_pose",
                "pose": pose_name
            }
            
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.arm_cmd_pub.publish(msg)
            
            rospy.loginfo("Arm pose command sent: {}".format(pose_name))
            
        except Exception as e:
            rospy.logerr("Arm pose command error: {}".format(e))
            messagebox.showerror("Error", "Failed to send arm pose command: {}".format(e))

    def stop_arm(self):
        """Stop arm movement"""
        try:
            cmd_data = {"cmd": "stop"}
            
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.arm_cmd_pub.publish(msg)
            
            rospy.loginfo("Arm stop command sent")
            
        except Exception as e:
            rospy.logerr("Arm stop error: {}".format(e))
            messagebox.showerror("Error", "Failed to stop arm: {}".format(e))

    def start_demo_after_moveit(self):
        """Legacy method - no longer needed"""
        pass

    def stop_moveit_after_demo(self):
        """Legacy method - no longer needed"""
        pass

    def send_arm_command(self, action):
        """Legacy method - redirect to new interface"""
        if action == "home":
            self.send_arm_pose_command("home")
        elif action == "pickup":
            self.send_arm_pose_command("pick")
        elif action == "place":
            self.send_arm_pose_command("ready")
        else:
            self.send_arm_pose_command(action)
    
    def emergency_stop(self):
        """Emergency stop"""
        try:
            msg = Bool()
            msg.data = True
            self.emergency_pub.publish(msg)
            
            rospy.loginfo("Emergency stop activated")
            messagebox.showwarning("Emergency Stop", "Emergency stop activated!")
            
        except Exception as e:
            rospy.logerr("Emergency stop error: {}".format(e))
            messagebox.showerror("Error", "Emergency stop failed: {}".format(e))
    
    def status_callback(self, msg):
        """System status callback"""
        try:
            data = json.loads(msg.data)
            self.robot_status.update({
                "mode": data.get("mode", "manual"),
                "emergency_stop": data.get("emergency_stop", False)
            })
            
            # Update GUI in main thread only if initialization is complete
            if self.initialization_complete:
                self.root.after(0, self.update_status_display)
            
        except Exception as e:
            rospy.logerr("Status callback error: {}".format(e))
    
    def linefollow_status_callback(self, msg):
        """Handle line following status updates"""
        try:
            status_data = json.loads(msg.data)
            
            linefollow_enabled = status_data.get('linefollow_enabled', False)
            camera_ready = status_data.get('camera_ready', False)
            preparing_camera = status_data.get('preparing_camera', False)
            
            # Update status display
            if preparing_camera:
                self.linefollow_status_var.set("Preparing Camera...")
            elif linefollow_enabled and camera_ready:
                self.linefollow_status_var.set("Active")
            elif linefollow_enabled:
                self.linefollow_status_var.set("Waiting for Camera")
            else:
                self.linefollow_status_var.set("Inactive")
            
            # Update robot status
            self.robot_status["linefollow_active"] = linefollow_enabled and camera_ready
            
        except json.JSONDecodeError as e:
            rospy.logdebug("Linefollow status JSON parsing failed: {}".format(e))
        except Exception as e:
            rospy.logdebug("Error processing linefollow status: {}".format(e))
    
    def arm_status_callback(self, msg):
        """Arm status callback"""
        try:
            data = json.loads(msg.data)
            arm_active = data.get("arm_active", False)
            current_motors = data.get("current_motors", [150, 150, 150])
            
            # Update robot status
            self.robot_status.update({
                "arm_active": arm_active
            })
            
            # Update arm status display
            if arm_active:
                self.arm_status_var.set("已激活")
            else:
                self.arm_status_var.set("未激活")
            
            # Update motor angle displays
            for i, angle in enumerate(current_motors):
                if i < len(self.motor_angle_vars):
                    self.motor_angle_vars[i].set("{:.0f}°".format(angle))
            
            # Update input fields if not currently focused (to avoid interfering with user input)
            for i, entry in enumerate(self.motor_entries):
                if entry != self.root.focus_get() and i < len(current_motors):
                    entry.delete(0, tk.END)  
                    entry.insert(0, str(int(current_motors[i])))
            
            # Update GUI in main thread only if initialization is complete
            if self.initialization_complete:
                self.root.after(0, self.update_status_display)
            
        except Exception as e:
            rospy.logerr("Arm status callback error: {}".format(e))
    
    def update_status_display(self):
        """Update status display widgets"""
        self.status_vars["mode"].set(self.robot_status["mode"])
        self.status_vars["moving"].set("Moving" if self.robot_status["moving"] else "Stopped")
        self.status_vars["linefollow"].set("Active" if self.robot_status["linefollow_active"] else "Inactive")
        self.status_vars["arm"].set("Active" if self.robot_status["arm_active"] else "Inactive")
        self.status_vars["emergency"].set("STOP!" if self.robot_status["emergency_stop"] else "Normal")
    
    def ros_spin(self):
        """ROS spinning in separate thread"""
        rospy.spin()
    
    def on_closing(self):
        """Handle window closing"""
        rospy.loginfo("Shutting down TransBot GUI...")
        rospy.signal_shutdown("GUI closed")
        self.root.destroy()

    def on_camera_pitch_change(self, value):
        """Handle camera pitch slider change - DEPRECATED: 现在使用数字输入框"""
        pass
    
    def on_camera_yaw_change(self, value):
        """Handle camera yaw slider change - DEPRECATED: 现在使用数字输入框"""
        pass
    
    def set_linefollow_position(self):
        """Set camera to line following position"""
        self.send_camera_command("linefollow_position")
        self.pitch_entry.delete(0, tk.END)
        self.pitch_entry.insert(0, str(self.LINEFOLLOW_PITCH))
        self.yaw_entry.delete(0, tk.END)
        self.yaw_entry.insert(0, str(self.LINEFOLLOW_YAW))
    
    def set_camera_center(self):
        """Set camera to center position"""
        self.send_camera_command("center")
        center_pitch = (10 + 140) // 2  # 75度
        self.pitch_entry.delete(0, tk.END)
        self.pitch_entry.insert(0, str(center_pitch))
        self.yaw_entry.delete(0, tk.END)
        self.yaw_entry.insert(0, str(self.LINEFOLLOW_YAW))
    
    def camera_up(self):
        """Move camera up"""
        self.send_camera_command("up", {"step": 10})
    
    def camera_down(self):
        """Move camera down"""
        self.send_camera_command("down", {"step": 10})
    
    def camera_left(self):
        """Move camera left"""
        self.send_camera_command("left", {"step": 10})
    
    def camera_right(self):
        """Move camera right"""
        self.send_camera_command("right", {"step": 10})
    
    def send_camera_command(self, cmd_type, extra_params=None):
        """Send camera control command"""
        try:
            cmd_data = {"cmd": cmd_type}
            if extra_params:
                cmd_data.update(extra_params)
            
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.camera_cmd_pub.publish(msg)
            
            rospy.loginfo("Camera command sent: {}".format(cmd_type))
            
        except Exception as e:
            rospy.logerr("Camera command error: {}".format(e))
    
    def send_camera_pitch_command(self, pitch):
        """Send specific camera pitch command"""
        try:
            cmd_data = {
                "cmd": "set_pitch",
                "angle": pitch
            }
            
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.camera_cmd_pub.publish(msg)
            
            rospy.loginfo("Camera pitch command sent: {} degrees".format(pitch))
            
        except Exception as e:
            rospy.logerr("Camera pitch command error: {}".format(e))
    
    def send_camera_yaw_command(self, yaw):
        """Send specific camera yaw command"""
        try:
            cmd_data = {
                "cmd": "set_yaw",
                "angle": yaw
            }
            
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.camera_cmd_pub.publish(msg)
            
            rospy.loginfo("Camera yaw command sent: {} degrees".format(yaw))
            
        except Exception as e:
            rospy.logerr("Camera yaw command error: {}".format(e))

    def camera_status_callback(self, msg):
        """Handle camera status updates"""
        try:
            status_data = json.loads(msg.data)
            self.camera_status.update(status_data)
            
            # Update GUI display
            current_pitch = status_data.get('current_pitch', self.LINEFOLLOW_PITCH)
            current_yaw = status_data.get('current_yaw', self.LINEFOLLOW_YAW)
            is_moving = status_data.get('is_moving', False)
            
            # Update angle displays - 使用"度"而不是度数符号
            if is_moving:
                self.camera_pitch_var.set("{}度 (Moving)".format(current_pitch))
                self.camera_yaw_var.set("{}度 (Moving)".format(current_yaw))
            else:
                self.camera_pitch_var.set("{}度".format(current_pitch))
                self.camera_yaw_var.set("{}度".format(current_yaw))
            
            # Update input fields if not actively being edited
            if not is_moving:
                # 只在输入框没有焦点时更新，避免用户输入时被覆盖
                if self.pitch_entry != self.root.focus_get():
                    self.pitch_entry.delete(0, tk.END)
                    self.pitch_entry.insert(0, str(current_pitch))
                if self.yaw_entry != self.root.focus_get():
                    self.yaw_entry.delete(0, tk.END)
                    self.yaw_entry.insert(0, str(current_yaw))
                
        except json.JSONDecodeError as e:
            rospy.logdebug("Camera status JSON parsing failed: {}".format(e))
        except Exception as e:
            rospy.logdebug("Error processing camera status: {}".format(e))

    def setup_emergency_controls(self, parent):
        """Setup emergency control"""
        emergency_frame = tk.Frame(parent, bg='#34495e')
        emergency_frame.pack(fill='both', expand=True, padx=20, pady=50)
        
        warning_label = tk.Label(emergency_frame, text="紧急停止", 
                                font=('Arial', 24, 'bold'), fg='#e74c3c', bg='#34495e')
        warning_label.pack(pady=30)
        
        self.emergency_button = tk.Button(emergency_frame, text="EMERGENCY STOP", 
                                         font=('Arial', 20, 'bold'), 
                                         bg='#e74c3c', fg='white', 
                                         command=self.emergency_stop,
                                         width=20, height=5)
        self.emergency_button.pack(pady=30)
        
        info_label = tk.Label(emergency_frame, text="点击此按钮立即停止所有机器人动作", 
                             font=('Arial', 12), fg='white', bg='#34495e')
        info_label.pack()

    def set_pitch_from_entry(self):
        """从输入框设置俯仰角度"""
        try:
            pitch = int(self.pitch_entry.get())
            if 10 <= pitch <= 140:  # 验证角度范围
                self.send_camera_pitch_command(pitch)
            else:
                messagebox.showerror("角度错误", "俯仰角度必须在10-140度之间")
        except ValueError:
            messagebox.showerror("输入错误", "请输入有效的数字")
        except Exception as e:
            rospy.logerr("Set pitch from entry error: {}".format(e))
    
    def set_yaw_from_entry(self):
        """从输入框设置水平角度"""
        try:
            yaw = int(self.yaw_entry.get())
            if 0 <= yaw <= 180:  # 验证角度范围
                self.send_camera_yaw_command(yaw)
            else:
                messagebox.showerror("角度错误", "水平角度必须在0-180度之间")
        except ValueError:
            messagebox.showerror("输入错误", "请输入有效的数字")
        except Exception as e:
            rospy.logerr("Set yaw from entry error: {}".format(e))

    def update_linefollow_speed(self):
        """Update line following speed"""
        try:
            new_speed = self.linefollow_speed.get()
            rospy.loginfo("Updating linefollow speed to: {}".format(new_speed))
            
            # 发送速度更新命令
            cmd_data = {
                "cmd": "set_speed",
                "speed": new_speed
            }
            
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.linefollow_control_pub.publish(msg)
            
            rospy.loginfo("Linefollow speed update command sent: {}".format(new_speed))
            
        except Exception as e:
            rospy.logerr("Update linefollow speed error: {}".format(e))
            self.show_error_dialog("速度更新错误", str(e))

    def move_to_motor_angles(self):
        """从输入框获取电机角度并发送移动命令"""
        try:
            motors = []
            for i, entry in enumerate(self.motor_entries):
                angle = int(entry.get())
                if 0 <= angle <= 300:
                    motors.append(angle)
                else:
                    messagebox.showerror("角度错误", "电机{}角度必须在0-300度之间".format(i+1))
                    return
            
            rospy.loginfo("Moving to motor angles: {}".format(motors))
            self.send_arm_motors_command(motors)
            
        except ValueError:
            messagebox.showerror("输入错误", "请输入有效的数字")
        except Exception as e:
            rospy.logerr("Move to motor angles error: {}".format(e))
            messagebox.showerror("移动错误", str(e))
    
    def get_current_motor_angles(self):
        """获取当前电机角度"""
        try:
            cmd_data = {"cmd": "get_current_motors"}
            
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.arm_cmd_pub.publish(msg)
            
            rospy.loginfo("Get current motor angles command sent")
            
        except Exception as e:
            rospy.logerr("Get current motor angles error: {}".format(e))
            messagebox.showerror("获取角度错误", str(e))
    
    def send_arm_motors_command(self, motors):
        """Send direct motor angles command"""
        try:
            cmd_data = {
                "cmd": "move_motors",
                "motors": motors
            }
            
            msg = String()
            msg.data = json.dumps(cmd_data)
            self.arm_cmd_pub.publish(msg)
            
            rospy.loginfo("Arm motors command sent: {}".format(motors))
            
        except Exception as e:
            rospy.logerr("Arm motors command error: {}".format(e))
            messagebox.showerror("Error", "Failed to send arm motors command: {}".format(e))

def main():
    try:
        # Create tkinter root
        root = tk.Tk()
        
        # Create GUI application
        app = TransBotGUI(root)
        
        # Handle window closing
        root.protocol("WM_DELETE_WINDOW", app.on_closing)
        
        rospy.loginfo("TransBot GUI ready")
        
        # Start GUI main loop
        root.mainloop()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("TransBot GUI interrupted by user")
    except Exception as e:
        rospy.logerr(f"TransBot GUI runtime error: {e}")

if __name__ == '__main__':
    main() 