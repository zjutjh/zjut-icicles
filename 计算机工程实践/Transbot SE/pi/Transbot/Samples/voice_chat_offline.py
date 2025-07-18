#!/usr/bin/env python3
#coding=utf-8

import subprocess
import wave
import time
import os
import threading
from aip import AipSpeech

# ==================== 配置部分 ====================
print("[初始化] 开始离线语音对话系统初始化...")

# 百度语音识别 API 配置（仅用于语音识别）
APP_ID = '90192389'
API_KEY = 'Fg9ZtQk3ymOq5ztw2QG0Ffiu'
SECRET_KEY = '8WXF8pgIxGzdvMikguWzx6AUx8mRtmaB'

# 语音设备配置
recommended_device = "default"  # 推荐的录音设备

# 初始化 AipSpeech 客户端（仅用于语音识别）
try:
    client = AipSpeech(APP_ID, API_KEY, SECRET_KEY)
    print("[初始化] ✓ 百度语音识别客户端初始化成功")
except Exception as e:
    print(f"[初始化] ✗ 百度语音识别客户端初始化失败: {e}")
    client = None

print("[初始化] 离线语音对话系统初始化完成")
print("=" * 50)

# ==================== 离线语音合成函数 ====================
def offline_speech_synthesis(text, voice_speed=5, voice_pitch=50, voice_volume=80):
    """
    使用espeak进行离线语音合成和播放
    
    Args:
        text: 要合成的文字
        voice_speed: 语速 (1-10, 默认5)
        voice_pitch: 音调 (0-99, 默认50)  
        voice_volume: 音量 (0-100, 默认80)
    
    Returns:
        bool: 成功返回True，失败返回False
    """
    print(f"\n[离线语音] 开始合成语音: '{text}'")
    print(f"[离线语音] 参数 - 语速:{voice_speed}, 音调:{voice_pitch}, 音量:{voice_volume}")
    
    try:
        # 构建espeak命令
        # -s: 语速 (words per minute, 默认175)
        # -p: 音调 (0-99)
        # -a: 音量 (0-200)
        # -v: 语音类型
        speed_wpm = int(voice_speed * 35)  # 转换为每分钟单词数
        
        command = [
            'espeak',
            '-s', str(speed_wpm),
            '-p', str(voice_pitch),
            '-a', str(voice_volume),
            '-v', 'zh',  # 中文语音
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

# ==================== 语音录制与识别函数 ====================
def record_and_recognize_speech(record_duration=4, device=None):
    """
    录制音频并转换为文字（使用百度API识别）
    
    Args:
        record_duration: 录音时长(秒)
        device: 录音设备，None时使用推荐设备
    
    Returns:
        识别的文字内容，失败返回None
    """
    if device is None:
        device = recommended_device
    
    print(f"\n[语音识别] 开始录音，时长: {record_duration}秒，设备: {device}")
    print("[语音识别] 请开始说话...")
    
    # 定义录音命令和参数
    audio_file = 'voice_record.wav'
    command = [
        'arecord',
        '-D', device,
        '-f', 'S16_LE',
        '-r', '16000',
        '-c', '1',
        '-d', str(record_duration),
        audio_file
    ]
    
    # 百度API参数
    options = {
        'dev_pid': 1536,  # 普通话输入法模型
        'lm_id': 0        # 默认语言模型
    }
    
    wf = None
    
    try:
        # 执行录音
        print(f"[语音识别] 执行命令: {' '.join(command)}")
        start_time = time.time()
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate()
        record_time = time.time() - start_time
        print(f"[语音识别] 录音耗时: {record_time:.2f}秒")
        
        if process.returncode != 0:
            print(f"[语音识别] ✗ 录音失败: {stderr.decode()}")
            return None
            
        print("[语音识别] ✓ 录音完成")
        
        # 检查录音文件
        if not os.path.exists(audio_file):
            print("[语音识别] ✗ 录音文件不存在")
            return None
            
        file_size = os.path.getsize(audio_file)
        print(f"[语音识别] 录音文件大小: {file_size} bytes")
        
        if file_size < 1000:
            print("[语音识别] ⚠ 录音文件过小，可能录音失败")
            return None
        
        # 分析音频文件
        try:
            wf = wave.open(audio_file, 'rb')
            channels = wf.getnchannels()
            sample_width = wf.getsampwidth()
            framerate = wf.getframerate()
            frames = wf.getnframes()
            duration = frames / float(framerate)
            
            print(f"[语音识别] 音频信息:")
            print(f"    声道数: {channels}")
            print(f"    位深: {sample_width * 8} bit")
            print(f"    采样率: {framerate} Hz")
            print(f"    帧数: {frames}")
            print(f"    实际时长: {duration:.2f} 秒")
            
            # 检查音频质量
            if duration < 0.5:
                print("[语音识别] ⚠ 音频时长过短，可能影响识别效果")
            elif duration > record_duration + 1:
                print("[语音识别] ⚠ 音频时长异常")
            
            # 读取音频数据
            FORMAT = 'wav' if sample_width == 2 else 'pcm'
            audio_data = wf.readframes(frames)
            print(f"[语音识别] 音频数据大小: {len(audio_data)} bytes，格式: {FORMAT}")
            
        except Exception as e:
            print(f"[语音识别] ✗ 音频文件分析失败: {e}")
            return None
        
        # 调用百度语音识别API
        print("[语音识别] 开始调用百度语音识别API...")
        api_start_time = time.time()
        
        if not client:
            print("[语音识别] ✗ 百度API客户端未初始化")
            return None
        
        try:
            result = client.asr(audio_data, FORMAT, framerate, options)
            api_time = time.time() - api_start_time
            print(f"[语音识别] API调用耗时: {api_time:.2f}秒")
            
        except Exception as e:
            print(f"[语音识别] ✗ API调用异常: {e}")
            return None
        
        # 处理识别结果
        if isinstance(result, dict):
            if 'result' in result and result['result']:
                recognized_text = result['result'][0]
                print(f"[语音识别] ✓ 识别成功: '{recognized_text}'")
                return recognized_text
            else:
                error_code = result.get('err_no', 'unknown')
                error_msg = result.get('err_msg', '未知错误')
                print(f"[语音识别] ✗ 识别失败 (错误码: {error_code}): {error_msg}")
                return None
        else:
            print(f"[语音识别] ✗ 返回数据格式异常: {type(result)}")
            return None
            
    except Exception as e:
        print(f"[语音识别] ✗ 录音识别过程异常: {e}")
        return None
    finally:
        if wf:
            wf.close()
        # 清理临时文件
        if os.path.exists(audio_file):
            try:
                os.remove(audio_file)
                print("[语音识别] 临时文件已清理")
            except:
                pass

# ==================== 智能对话生成函数 ====================
def generate_chat_response(user_input):
    """
    生成对话回复（简单规则版本）
    
    Args:
        user_input: 用户输入的文字
    
    Returns:
        生成的回复文字
    """
    user_input = user_input.lower().strip()
    
    # 简单的规则对话系统
    responses = {
        "你好": "你好！很高兴见到你！",
        "再见": "再见！祝你有美好的一天！",
        "谢谢": "不客气！有什么需要帮助的吗？",
        "时间": f"现在时间是 {time.strftime('%H点%M分')}",
        "日期": f"今天是 {time.strftime('%Y年%m月%d日')}",
        "天气": "抱歉，我无法获取实时天气信息，建议您查看天气应用。",
        "介绍": "我是一个语音对话机器人，可以和您进行简单的对话交流。",
        "功能": "我可以进行语音识别、语音合成、简单对话等功能。",
        "帮助": "您可以说'你好'、'时间'、'日期'、'介绍'等，我会给您回复。"
    }
    
    # 关键词匹配
    for keyword, response in responses.items():
        if keyword in user_input:
            return response
    
    # 默认回复
    return "我听到了您说的话，但是我不太理解。您可以说'帮助'来了解我能做什么。"

# ==================== 主要对话函数 ====================
def voice_chat_session():
    """
    语音对话会话
    """
    print("\n" + "=" * 60)
    print("[对话开始] 欢迎使用离线语音对话系统！")
    print("[对话提示] 说'退出'或'再见'可以结束对话")
    print("=" * 60)
    
    # 播放欢迎语音
    offline_speech_synthesis("你好！欢迎使用语音对话系统！请开始说话。", voice_speed=6)
    
    conversation_count = 0
    
    while True:
        try:
            conversation_count += 1
            print(f"\n--- 第 {conversation_count} 轮对话 ---")
            
            # 语音识别
            user_input = record_and_recognize_speech(record_duration=5)
            
            if not user_input:
                offline_speech_synthesis("抱歉，我没有听清楚，请再说一遍。")
                continue
            
            print(f"[用户输入] {user_input}")
            
            # 检查退出条件
            if any(word in user_input.lower() for word in ['退出', '再见', '结束', '停止']):
                response = "好的，再见！感谢使用语音对话系统！"
                print(f"[系统回复] {response}")
                offline_speech_synthesis(response)
                break
            
            # 生成回复
            response = generate_chat_response(user_input)
            print(f"[系统回复] {response}")
            
            # 语音播报回复
            offline_speech_synthesis(response, voice_speed=6)
            
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
    print("[状态检查] 离线语音对话系统健康检查")
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
    
    # 检查百度语音识别API
    print("\n[状态检查] === 百度语音识别API检查 ===")
    if client:
        print("[状态检查] ✓ 百度语音识别客户端已初始化")
        print("[状态检查] ℹ 注意：仅使用百度API进行语音识别，语音合成使用离线方案")
    else:
        print("[状态检查] ✗ 百度语音识别客户端未初始化")
    
    # 检查录音功能
    print("\n[状态检查] === 录音功能测试 ===")
    print("[状态检查] 请说话进行录音测试（2秒）...")
    try:
        test_text = record_and_recognize_speech(record_duration=2)
        if test_text:
            print(f"[状态检查] ✓ 录音识别功能正常，识别结果: '{test_text}'")
        else:
            print("[状态检查] ⚠ 录音识别未获得结果，可能是环境太安静或设备问题")
    except Exception as e:
        print(f"[状态检查] ✗ 录音功能测试异常: {e}")
    
    # 检查对话生成
    print("\n[状态检查] === 对话生成测试 ===")
    try:
        test_response = generate_chat_response("你好")
        print(f"[状态检查] ✓ 对话生成功能正常，测试回复: '{test_response}'")
    except Exception as e:
        print(f"[状态检查] ✗ 对话生成测试异常: {e}")
    
    print("\n" + "=" * 60)
    print("[状态检查] 系统健康检查完成")
    print("=" * 60)

# ==================== 主程序 ====================
if __name__ == "__main__":
    try:
        print("离线语音对话系统 V1.0")
        print("功能：语音识别(百度API) + 离线语音合成(espeak) + 智能对话")
        
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