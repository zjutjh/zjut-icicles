#!/usr/bin/env python3
#coding=utf-8

import subprocess
import wave
import time
import os
import threading

# ==================== 配置部分 ====================
print("[初始化] 开始语音识别系统初始化...")

# USB麦克风设备配置
usb_mic_device = "hw:0,0"  # USB麦克风设备

print("[初始化] 语音识别系统初始化完成")
print("=" * 50)

# ==================== USB麦克风录音函数 ====================
def record_speech_usb_mic(record_duration=4):
    """
    使用USB麦克风录音
    
    Args:
        record_duration: 录音时长(秒)
    
    Returns:
        录音文件名，失败返回None
    """
    print(f"\n[语音录制] 开始录音，时长: {record_duration}秒")
    print("[语音录制] 请对着USB麦克风说话...")
    
    # 定义录音命令和参数
    audio_file = f'voice_record_{int(time.time())}.wav'
    command = [
        'arecord',
        '-D', usb_mic_device,
        '-f', 'S16_LE',
        '-r', '48000',  # USB麦克风支持48kHz
        '-c', '1',
        '-d', str(record_duration),
        audio_file
    ]
    
    try:
        # 执行录音
        print(f"[语音录制] 执行命令: {' '.join(command)}")
        start_time = time.time()
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate()
        record_time = time.time() - start_time
        print(f"[语音录制] 录音耗时: {record_time:.2f}秒")
        
        if process.returncode != 0:
            print(f"[语音录制] ✗ 录音失败: {stderr.decode()}")
            return None
            
        print("[语音录制] ✓ 录音完成")
        
        # 检查录音文件
        if not os.path.exists(audio_file):
            print("[语音录制] ✗ 录音文件不存在")
            return None
            
        file_size = os.path.getsize(audio_file)
        print(f"[语音录制] 录音文件: {audio_file} ({file_size} bytes)")
        
        if file_size < 10000:
            print("[语音录制] ⚠ 录音文件较小，可能环境太安静")
            
        return audio_file
            
    except Exception as e:
        print(f"[语音录制] ✗ 录音过程异常: {e}")
        return None

# ==================== 音频分析函数 ====================
def analyze_audio_file(audio_file):
    """
    分析音频文件的基本信息
    
    Args:
        audio_file: 音频文件路径
    
    Returns:
        dict: 音频信息字典
    """
    try:
        with wave.open(audio_file, 'rb') as wf:
            channels = wf.getnchannels()
            sample_width = wf.getsampwidth()
            framerate = wf.getframerate()
            frames = wf.getnframes()
            duration = frames / float(framerate)
            
            audio_info = {
                'channels': channels,
                'sample_width': sample_width,
                'framerate': framerate,
                'frames': frames,
                'duration': duration
            }
            
            print(f"[音频分析] 音频信息:")
            print(f"    文件: {audio_file}")
            print(f"    声道数: {channels}")
            print(f"    位深: {sample_width * 8} bit")
            print(f"    采样率: {framerate} Hz")
            print(f"    帧数: {frames}")
            print(f"    时长: {duration:.2f} 秒")
            
            return audio_info
            
    except Exception as e:
        print(f"[音频分析] ✗ 音频文件分析失败: {e}")
        return None

# ==================== 简单语音识别模拟函数 ====================
def simple_voice_recognition(audio_file):
    """
    简单的语音识别模拟（通过键盘输入）
    
    Args:
        audio_file: 音频文件路径
    
    Returns:
        识别的文字内容
    """
    print(f"\n[语音识别] 分析音频文件: {audio_file}")
    
    # 分析音频文件
    audio_info = analyze_audio_file(audio_file)
    if not audio_info:
        return None
    
    # 根据音频时长和质量判断是否有语音
    if audio_info['duration'] > 0.5 and os.path.getsize(audio_file) > 50000:
        print("[语音识别] 检测到语音活动")
        print("[语音识别] 请输入您刚才说的内容（用于识别确认）:")
        user_input = input("您说了什么: ").strip()
        
        if user_input:
            print(f"[语音识别] ✓ 识别结果: '{user_input}'")
            return user_input
        else:
            print("[语音识别] ✗ 未输入内容")
            return None
    else:
        print("[语音识别] ⚠ 未检测到有效语音或音频质量不佳")
        return None

# ==================== 智能响应函数 ====================
def generate_response(recognized_text):
    """
    根据识别的文字生成响应
    
    Args:
        recognized_text: 识别的文字
    
    Returns:
        响应文字
    """
    if not recognized_text:
        return "未能识别到有效语音"
    
    text = recognized_text.lower().strip()
    
    # 简单的响应规则
    responses = {
        "你好": "你好！很高兴与您对话！",
        "hello": "Hello! Nice to meet you!",
        "时间": f"现在时间是 {time.strftime('%H点%M分')}",
        "time": f"Current time is {time.strftime('%H:%M')}",
        "日期": f"今天是 {time.strftime('%Y年%m月%d日')}",
        "date": f"Today is {time.strftime('%Y-%m-%d')}",
        "测试": "语音识别测试成功！",
        "test": "Voice recognition test successful!",
        "系统": "语音识别系统运行正常",
        "帮助": "您可以说：你好、时间、日期、测试等词语",
        "help": "You can say: hello, time, date, test, etc."
    }
    
    # 关键词匹配
    for keyword, response in responses.items():
        if keyword in text:
            return response
    
    # 默认响应
    if any(char in text for char in '你我他她它'):
        return f"我听到您说了：'{recognized_text}'，但是我还不能完全理解这句话的含义。"
    else:
        return f"I heard you say: '{recognized_text}', but I don't fully understand the meaning yet."

# ==================== 语音识别会话函数 ====================
def voice_recognition_session():
    """
    语音识别会话
    """
    print("\n" + "=" * 60)
    print("[会话开始] 欢迎使用语音识别系统！")
    print("[使用说明] 系统会录制您的语音，然后请您确认说话内容")
    print("[退出方式] 说'退出'、'再见'、'goodbye'或'bye'可以结束会话")
    print("=" * 60)
    
    session_count = 0
    
    while True:
        try:
            session_count += 1
            print(f"\n--- 第 {session_count} 次语音识别 ---")
            
            # 录音
            audio_file = record_speech_usb_mic(record_duration=5)
            
            if not audio_file:
                print("[系统提示] 录音失败，请重试")
                continue
            
            # 语音识别
            recognized_text = simple_voice_recognition(audio_file)
            
            if not recognized_text:
                print("[系统提示] 未识别到有效语音，请重试")
                # 清理临时文件
                if os.path.exists(audio_file):
                    os.remove(audio_file)
                continue
            
            print(f"[识别结果] {recognized_text}")
            
            # 检查退出条件
            exit_words = ['退出', '再见', '结束', '停止', 'goodbye', 'bye', 'exit', 'quit']
            if any(word in recognized_text.lower() for word in exit_words):
                print("\n[会话结束] 感谢使用语音识别系统！")
                # 清理临时文件
                if os.path.exists(audio_file):
                    os.remove(audio_file)
                break
            
            # 生成响应
            response = generate_response(recognized_text)
            print(f"[系统响应] {response}")
            
            # 清理临时文件
            if os.path.exists(audio_file):
                os.remove(audio_file)
                print("[文件管理] 临时录音文件已清理")
            
        except KeyboardInterrupt:
            print("\n[会话结束] 用户中断会话")
            break
        except Exception as e:
            print(f"[会话错误] 会话过程出现异常: {e}")
            continue
    
    print("\n" + "=" * 60)
    print("[会话结束] 语音识别系统已退出")
    print("=" * 60)

# ==================== 系统测试函数 ====================
def system_test():
    """
    系统功能测试
    """
    print("\n" + "=" * 60)
    print("[系统测试] 语音识别系统功能测试")
    print("=" * 60)
    
    # 测试USB麦克风
    print("\n[测试项目] === USB麦克风录音测试 ===")
    print("[测试说明] 请对着USB麦克风说话（3秒）...")
    try:
        test_audio = record_speech_usb_mic(record_duration=3)
        if test_audio:
            print("[测试结果] ✓ USB麦克风录音功能正常")
            
            # 测试音频分析
            print("\n[测试项目] === 音频分析测试 ===")
            audio_info = analyze_audio_file(test_audio)
            if audio_info:
                print("[测试结果] ✓ 音频分析功能正常")
            else:
                print("[测试结果] ✗ 音频分析功能异常")
            
            # 测试语音识别
            print("\n[测试项目] === 语音识别测试 ===")
            recognized = simple_voice_recognition(test_audio)
            if recognized:
                print(f"[测试结果] ✓ 语音识别功能正常，识别结果: '{recognized}'")
                
                # 测试响应生成
                print("\n[测试项目] === 响应生成测试 ===")
                response = generate_response(recognized)
                print(f"[测试结果] ✓ 响应生成功能正常: '{response}'")
            else:
                print("[测试结果] ⚠ 语音识别未获得结果")
            
            # 清理测试文件
            if os.path.exists(test_audio):
                os.remove(test_audio)
                print("[文件管理] 测试文件已清理")
                
        else:
            print("[测试结果] ✗ USB麦克风录音功能异常")
            
    except Exception as e:
        print(f"[测试异常] 系统测试过程异常: {e}")
    
    print("\n" + "=" * 60)
    print("[系统测试] 测试完成")
    print("=" * 60)

# ==================== 设备检查函数 ====================
def check_devices():
    """
    检查音频设备状态
    """
    print("\n" + "=" * 60)
    print("[设备检查] 音频设备状态检查")
    print("=" * 60)
    
    # 检查音频卡
    print("\n[检查项目] === 音频卡列表 ===")
    try:
        with open('/proc/asound/cards', 'r') as f:
            cards_info = f.read()
            print(cards_info)
    except Exception as e:
        print(f"[检查错误] 无法读取音频卡信息: {e}")
    
    # 检查USB麦克风
    print("\n[检查项目] === USB麦克风设备 ===")
    try:
        with open('/proc/asound/card0/stream0', 'r') as f:
            stream_info = f.read()
            print(stream_info)
    except Exception as e:
        print(f"[检查错误] 无法读取USB设备信息: {e}")
    
    # 测试录音设备
    print("\n[检查项目] === 录音设备测试 ===")
    try:
        result = subprocess.run(['arecord', '-l'], capture_output=True, text=True)
        if result.returncode == 0:
            print("[设备状态] ✓ 录音设备列表:")
            print(result.stdout)
        else:
            print(f"[设备状态] ✗ 录音设备检查失败: {result.stderr}")
    except Exception as e:
        print(f"[检查错误] 录音设备检查异常: {e}")
    
    print("=" * 60)

# ==================== 主程序 ====================
if __name__ == "__main__":
    try:
        print("语音识别系统 V1.0")
        print("功能：USB麦克风录音 + 语音识别 + 智能响应")
        print("说明：本系统仅支持语音识别，不支持语音播放")
        
        while True:
            print("\n请选择操作:")
            print("1. 系统功能测试")
            print("2. 设备状态检查") 
            print("3. 开始语音识别会话")
            print("4. 退出程序")
            
            choice = input("请输入选择 (1-4): ").strip()
            
            if choice == '1':
                system_test()
            elif choice == '2':
                check_devices()
            elif choice == '3':
                voice_recognition_session()
            elif choice == '4':
                print("程序退出，再见！")
                break
            else:
                print("无效选择，请重新输入")
                
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序运行异常: {e}")
    finally:
        print("程序结束") 