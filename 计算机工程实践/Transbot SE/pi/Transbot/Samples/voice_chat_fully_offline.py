#!/usr/bin/env python3
#coding=utf-8

import subprocess
import wave
import time
import os
import threading
import speech_recognition as sr
import tempfile

# ==================== 配置部分 ====================
print("[初始化] 开始完全离线语音对话系统初始化...")

# 语音设备配置
recommended_device = "default"  # 推荐的录音设备

# 初始化语音识别器
try:
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()
    print("[初始化] ✓ 离线语音识别器初始化成功")
except Exception as e:
    print(f"[初始化] ✗ 离线语音识别器初始化失败: {e}")
    recognizer = None
    microphone = None

print("[初始化] 完全离线语音对话系统初始化完成")
print("=" * 50)

# ==================== 离线语音合成函数 ====================
def offline_speech_synthesis(text, voice_speed=5, voice_pitch=50, voice_volume=80, language='zh'):
    """
    使用espeak进行离线语音合成和播放
    
    Args:
        text: 要合成的文字
        voice_speed: 语速 (1-10, 默认5)
        voice_pitch: 音调 (0-99, 默认50)  
        voice_volume: 音量 (0-100, 默认80)
        language: 语言 ('zh'中文, 'en'英文)
    
    Returns:
        bool: 成功返回True，失败返回False
    """
    print(f"\n[离线语音] 开始合成语音: '{text}'")
    print(f"[离线语音] 参数 - 语速:{voice_speed}, 音调:{voice_pitch}, 音量:{voice_volume}")
    
    try:
        # 构建espeak命令
        speed_wpm = int(voice_speed * 35)  # 转换为每分钟单词数
        
        command = [
            'espeak',
            '-s', str(speed_wpm),
            '-p', str(voice_pitch),
            '-a', str(voice_volume),
            '-v', language,  # 语言设置
            text
        ]
        
        print(f"[离线语音] 执行命令: {' '.join(command)}")
        start_time = time.time()
        
        # 执行语音合成
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        stdout, stderr = process.communicate()
        
        synthesis_time = time.time() - start_time
        print(f"[离线语音] 语音合成耗时: {synthesis_time:.2f}秒")
        
        if process.returncode == 0:
            print("[离线语音] ✓ 语音合成播放成功")
            return True
        else:
            print(f"[离线语音] ✗ 语音合成失败: {stderr.decode()}")
            return False
            
    except Exception as e:
        print(f"[离线语音] ✗ 语音合成异常: {e}")
        return False

# ==================== 备用语音识别函数 ====================
def fallback_record_and_recognize_speech(record_duration=4, device=None):
    """
    回退方案：使用arecord录音 + 键盘输入
    
    Args:
        record_duration: 录音时长(秒)
        device: 录音设备，None时使用推荐设备
    
    Returns:
        识别的文字内容，失败返回None
    """
    if device is None:
        device = recommended_device
    
    print(f"\n[备用识别] 开始录音，时长: {record_duration}秒，设备: {device}")
    print("[备用识别] 请开始说话...")
    
    # 定义录音命令和参数
    audio_file = 'voice_record_fallback.wav'
    command = [
        'arecord',
        '-D', device,
        '-f', 'S16_LE',
        '-r', '16000',
        '-c', '1',
        '-d', str(record_duration),
        audio_file
    ]
    
    try:
        # 执行录音
        print(f"[备用识别] 执行命令: {' '.join(command)}")
        start_time = time.time()
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate()
        record_time = time.time() - start_time
        print(f"[备用识别] 录音耗时: {record_time:.2f}秒")
        
        if process.returncode != 0:
            print(f"[备用识别] ✗ 录音失败: {stderr.decode()}")
            return None
            
        print("[备用识别] ✓ 录音完成")
        
        # 检查录音文件
        if not os.path.exists(audio_file):
            print("[备用识别] ✗ 录音文件不存在")
            return None
            
        file_size = os.path.getsize(audio_file)
        print(f"[备用识别] 录音文件大小: {file_size} bytes")
        
        if file_size > 30000:
            print("[备用识别] 检测到语音活动，请通过键盘输入您刚才说的话:")
            keyboard_input = input("您说了什么: ").strip()
            if keyboard_input:
                print(f"[备用识别] ✓ 接收到输入: '{keyboard_input}'")
                return keyboard_input
        
        print("[备用识别] ✗ 未检测到有效语音")
        return None
            
    except Exception as e:
        print(f"[备用识别] ✗ 录音识别过程异常: {e}")
        return None
    finally:
        # 清理临时文件
        if os.path.exists(audio_file):
            try:
                os.remove(audio_file)
                print("[备用识别] 临时文件已清理")
            except:
                pass

# ==================== 智能对话生成函数 ====================
def generate_chat_response(user_input):
    """
    生成对话回复
    """
    user_input = user_input.lower().strip()
    
    responses = {
        "你好": "你好！很高兴见到你！",
        "hello": "Hello! Nice to meet you!",
        "hi": "Hi there! How can I help you?",
        "再见": "再见！祝你有美好的一天！",
        "goodbye": "Goodbye! Have a great day!",
        "bye": "Bye! See you later!",
        "谢谢": "不客气！有什么需要帮助的吗？",
        "thank": "You're welcome! Is there anything I can help you with?",
        "时间": f"现在时间是 {time.strftime('%H点%M分')}",
        "time": f"Current time is {time.strftime('%H:%M')}",
        "日期": f"今天是 {time.strftime('%Y年%m月%d日')}",
        "date": f"Today is {time.strftime('%Y-%m-%d')}",
        "天气": "抱歉，我无法获取实时天气信息。",
        "weather": "Sorry, I cannot get real-time weather information.",
        "介绍": "我是一个离线语音对话机器人。",
        "introduce": "I am an offline voice chat robot.",
        "帮助": "您可以说'你好'、'时间'、'日期'等。",
        "help": "You can say 'hello', 'time', 'date', etc.",
        "test": "Test successful! The system is working properly.",
        "测试": "测试成功！系统运行正常。"
    }
    
    for keyword, response in responses.items():
        if keyword in user_input:
            return response
    
    if any(char in user_input for char in '你我他她它'):
        return "我听到了您说的话，但是我不太理解。您可以说'帮助'来了解我能做什么。"
    else:
        return "I heard what you said, but I don't quite understand. You can say 'help' to learn what I can do."

# ==================== 主要对话函数 ====================
def voice_chat_session():
    """
    语音对话会话
    """
    print("\n" + "=" * 60)
    print("[对话开始] 欢迎使用完全离线语音对话系统！")
    print("[对话提示] 说'退出'、'再见'、'goodbye'或'bye'可以结束对话")
    print("=" * 60)
    
    # 播放欢迎语音
    offline_speech_synthesis("你好！欢迎使用离线语音对话系统！请开始说话。", voice_speed=6)
    
    conversation_count = 0
    
    while True:
        try:
            conversation_count += 1
            print(f"\n--- 第 {conversation_count} 轮对话 ---")
            
            # 使用备用方案进行语音识别
            user_input = fallback_record_and_recognize_speech(record_duration=5)
            
            if not user_input:
                offline_speech_synthesis("抱歉，我没有听清楚，请再说一遍。")
                continue
            
            print(f"[用户输入] {user_input}")
            
            # 检查退出条件
            exit_words = ['退出', '再见', '结束', '停止', 'goodbye', 'bye', 'exit', 'quit']
            if any(word in user_input.lower() for word in exit_words):
                response = "好的，再见！感谢使用语音对话系统！"
                print(f"[系统回复] {response}")
                offline_speech_synthesis(response)
                break
            
            # 生成回复
            response = generate_chat_response(user_input)
            print(f"[系统回复] {response}")
            
            # 语音播报回复
            if any(char in response for char in '你我他她它是的了吗'):
                offline_speech_synthesis(response, voice_speed=6, language='zh')
            else:
                offline_speech_synthesis(response, voice_speed=6, language='en')
            
        except KeyboardInterrupt:
            print("\n[对话结束] 用户中断对话")
            offline_speech_synthesis("对话被中断，再见！")
            break
        except Exception as e:
            print(f"[对话错误] 对话过程出现异常: {e}")
            offline_speech_synthesis("出现了一些问题，让我们重新开始。")
            continue
    
    print("\n" + "=" * 60)
    print("[对话结束] 语音对话系统已退出")
    print("=" * 60)

# ==================== 系统状态检查 ====================
def system_health_check():
    """
    系统健康状态检查
    """
    print("\n" + "=" * 60)
    print("[状态检查] 完全离线语音对话系统健康检查")
    print("=" * 60)
    
    # 检查离线语音合成
    print("\n[状态检查] === 离线语音合成检查 ===")
    try:
        test_result = offline_speech_synthesis("系统测试", voice_speed=8)
        if test_result:
            print("[状态检查] ✓ 离线语音合成功能正常")
        else:
            print("[状态检查] ✗ 离线语音合成功能异常")
    except Exception as e:
        print(f"[状态检查] ✗ 离线语音合成测试异常: {e}")
    
    # 检查备用识别方案
    print("\n[状态检查] === 备用语音识别检查 ===")
    print("[状态检查] 请说话进行录音测试（3秒）...")
    try:
        test_text = fallback_record_and_recognize_speech(record_duration=3)
        if test_text:
            print(f"[状态检查] ✓ 备用识别方案正常，识别结果: '{test_text}'")
        else:
            print("[状态检查] ⚠ 备用识别方案未获得结果")
    except Exception as e:
        print(f"[状态检查] ✗ 备用识别测试异常: {e}")
    
    # 检查对话生成
    print("\n[状态检查] === 对话生成测试 ===")
    try:
        test_response_zh = generate_chat_response("你好")
        test_response_en = generate_chat_response("hello")
        print(f"[状态检查] ✓ 中文对话生成功能正常: '{test_response_zh}'")
        print(f"[状态检查] ✓ 英文对话生成功能正常: '{test_response_en}'")
    except Exception as e:
        print(f"[状态检查] ✗ 对话生成测试异常: {e}")
    
    print("\n" + "=" * 60)
    print("[状态检查] 系统健康检查完成")
    print("=" * 60)

# ==================== 设备检查函数 ====================
def check_audio_devices():
    """
    检查可用的音频设备
    """
    print("\n" + "=" * 60)
    print("[设备检查] 音频设备检查")
    print("=" * 60)
    
    # 检查麦克风设备
    try:
        print("\n[设备检查] === 可用麦克风设备 ===")
        mic_list = sr.Microphone.list_microphone_names()
        for i, microphone_name in enumerate(mic_list):
            print(f"    设备 {i}: {microphone_name}")
        
        if mic_list:
            print(f"[设备检查] ✓ 找到 {len(mic_list)} 个麦克风设备")
        else:
            print("[设备检查] ✗ 未找到麦克风设备")
            
    except Exception as e:
        print(f"[设备检查] ✗ 麦克风设备检查异常: {e}")
    
    # 检查录音设备（使用arecord）
    print("\n[设备检查] === 录音设备检查 (arecord) ===")
    try:
        result = subprocess.run(['arecord', '-l'], capture_output=True, text=True)
        if result.returncode == 0:
            print("[设备检查] ✓ arecord设备列表:")
            print(result.stdout)
        else:
            print(f"[设备检查] ✗ arecord设备检查失败: {result.stderr}")
    except Exception as e:
        print(f"[设备检查] ✗ arecord设备检查异常: {e}")
    
    print("=" * 60)

# ==================== 主程序 ====================
if __name__ == "__main__":
    try:
        print("完全离线语音对话系统 V1.0")
        print("功能：离线语音合成(espeak) + 语音录制 + 智能对话")
        
        while True:
            print("\n请选择操作:")
            print("1. 系统健康检查")
            print("2. 开始语音对话")
            print("3. 退出程序")
            
            choice = input("请输入选择 (1-3): ").strip()
            
            if choice == '1':
                system_health_check()
            elif choice == '2':
                voice_chat_session()
            elif choice == '3':
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