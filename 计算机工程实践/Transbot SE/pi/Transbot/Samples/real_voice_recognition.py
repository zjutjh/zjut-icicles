#!/usr/bin/env python3
#coding=utf-8

import subprocess
import wave
import time
import os
import json
import vosk

# ==================== 配置部分 ====================
print("[初始化] 开始真实语音识别系统初始化...")

# USB麦克风设备配置
usb_mic_device = "hw:0,0"  # USB麦克风设备

# Vosk模型路径
model_path = "vosk-model-small-cn-0.22"

# 初始化Vosk语音识别模型
try:
    if os.path.exists(model_path):
        print(f"[初始化] 加载Vosk中文语音识别模型: {model_path}")
        model = vosk.Model(model_path)
        print("[初始化] ✓ Vosk语音识别模型加载成功")
    else:
        print(f"[初始化] ✗ 语音识别模型不存在: {model_path}")
        print("[初始化] 请确保已下载并解压语音识别模型")
        model = None
except Exception as e:
    print(f"[初始化] ✗ Vosk语音识别模型加载失败: {e}")
    model = None

print("[初始化] 真实语音识别系统初始化完成")
print("=" * 50)

# ==================== USB麦克风录音函数 ====================
def record_speech_usb_mic(record_duration=4, save_file=True):
    """
    使用USB麦克风录音
    
    Args:
        record_duration: 录音时长(秒)
        save_file: 是否保存录音文件
    
    Returns:
        录音文件名，失败返回None
    """
    print(f"\n[语音录制] 开始录音，时长: {record_duration}秒")
    print("[语音录制] 请对着USB麦克风清晰说话...")
    
    # 定义录音命令和参数
    timestamp = int(time.time())
    if save_file:
        audio_file = f'voice_record_{timestamp}.wav'
    else:
        audio_file = f'temp_voice_{timestamp}.wav'
    
    command = [
        'arecord',
        '-D', usb_mic_device,
        '-f', 'S16_LE',
        '-r', '16000',  # Vosk推荐16kHz采样率
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
        
        if file_size < 5000:
            print("[语音录制] ⚠ 录音文件较小，可能环境太安静或录音时间过短")
            
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

# ==================== Vosk语音识别函数 ====================
def vosk_speech_recognition(audio_file):
    """
    使用Vosk进行语音识别
    
    Args:
        audio_file: 音频文件路径
    
    Returns:
        识别的文字内容
    """
    if not model:
        print("[语音识别] ✗ Vosk模型未加载")
        return None
    
    print(f"\n[语音识别] 开始识别音频文件: {audio_file}")
    
    try:
        # 分析音频文件
        audio_info = analyze_audio_file(audio_file)
        if not audio_info:
            return None
        
        # 检查音频格式是否符合Vosk要求
        if audio_info['framerate'] != 16000:
            print(f"[语音识别] ⚠ 音频采样率为{audio_info['framerate']}Hz，Vosk推荐16000Hz")
        
        if audio_info['channels'] != 1:
            print(f"[语音识别] ⚠ 音频为{audio_info['channels']}声道，Vosk要求单声道")
        
        # 初始化识别器
        rec = vosk.KaldiRecognizer(model, audio_info['framerate'])
        rec.SetWords(True)
        
        # 读取音频数据
        with wave.open(audio_file, 'rb') as wf:
            print("[语音识别] 正在处理音频数据...")
            
            results = []
            while True:
                data = wf.readframes(4000)
                if len(data) == 0:
                    break
                
                if rec.AcceptWaveform(data):
                    result = json.loads(rec.Result())
                    if result.get('text'):
                        results.append(result['text'])
            
            # 获取最终结果
            final_result = json.loads(rec.FinalResult())
            if final_result.get('text'):
                results.append(final_result['text'])
            
            # 合并所有识别结果
            if results:
                recognized_text = ' '.join(results).strip()
                print(f"[语音识别] ✓ 识别成功: '{recognized_text}'")
                return recognized_text
            else:
                print("[语音识别] ⚠ 未识别到任何文字")
                return None
                
    except Exception as e:
        print(f"[语音识别] ✗ 语音识别过程异常: {e}")
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
        "状态": "系统状态良好，语音识别功能正常",
        "帮助": "您可以说：你好、时间、日期、测试、状态等词语",
        "help": "You can say: hello, time, date, test, status, etc.",
        "天气": "抱歉，我无法获取天气信息",
        "再见": "再见！感谢使用语音识别系统！",
        "谢谢": "不客气！很高兴为您服务！"
    }
    
    # 关键词匹配
    for keyword, response in responses.items():
        if keyword in text:
            return response
    
    # 默认响应
    if any(char in text for char in '你我他她它'):
        return f"我听到您说了：'{recognized_text}'。这是一个很有趣的话题！"
    else:
        return f"I heard you say: '{recognized_text}'. That's interesting!"

# ==================== 语音识别会话函数 ====================
def voice_recognition_session():
    """
    语音识别会话
    """
    print("\n" + "=" * 60)
    print("[会话开始] 欢迎使用真实语音识别系统！")
    print("[使用说明] 系统会录制您的语音并自动识别转换为文字")
    print("[退出方式] 说'退出'、'再见'、'goodbye'或'bye'可以结束会话")
    print("[注意事项] 请靠近麦克风清晰说话，避免背景噪音")
    print("=" * 60)
    
    session_count = 0
    
    while True:
        try:
            session_count += 1
            print(f"\n--- 第 {session_count} 次语音识别 ---")
            
            # 录音
            audio_file = record_speech_usb_mic(record_duration=4, save_file=True)
            
            if not audio_file:
                print("[系统提示] 录音失败，请重试")
                continue
            
            # 语音识别
            recognized_text = vosk_speech_recognition(audio_file)
            
            if not recognized_text:
                print("[系统提示] 未识别到语音内容，请重试")
                print("[提示] 请确保:")
                print("  - 靠近麦克风说话")
                print("  - 说话声音清晰")
                print("  - 避免背景噪音")
                print("  - 说话时间超过1秒")
                continue
            
            print(f"[识别结果] {recognized_text}")
            
            # 检查退出条件
            exit_words = ['退出', '再见', '结束', '停止', 'goodbye', 'bye', 'exit', 'quit']
            if any(word in recognized_text.lower() for word in exit_words):
                print("\n[会话结束] 感谢使用语音识别系统！")
                print(f"[文件保存] 最后的录音文件已保存: {audio_file}")
                break
            
            # 生成响应
            response = generate_response(recognized_text)
            print(f"[系统响应] {response}")
            
            print(f"[文件保存] 录音文件已保存: {audio_file}")
            
        except KeyboardInterrupt:
            print("\n[会话结束] 用户中断会话")
            break
        except Exception as e:
            print(f"[会话错误] 会话过程出现异常: {e}")
            continue
    
    print("\n" + "=" * 60)
    print("[会话结束] 语音识别系统已退出")
    print("=" * 60)

# ==================== 单次识别测试函数 ====================
def single_recognition_test():
    """
    单次语音识别测试
    """
    print("\n" + "=" * 60)
    print("[单次测试] 语音识别单次测试")
    print("=" * 60)
    
    try:
        # 录音
        print("[测试步骤] 1. 录音")
        audio_file = record_speech_usb_mic(record_duration=3, save_file=True)
        
        if not audio_file:
            print("[测试结果] ✗ 录音失败")
            return
        
        # 语音识别
        print("\n[测试步骤] 2. 语音识别")
        recognized_text = vosk_speech_recognition(audio_file)
        
        if recognized_text:
            print(f"\n[测试结果] ✓ 识别成功!")
            print(f"[识别文本] {recognized_text}")
            print(f"[录音文件] {audio_file}")
            
            # 生成响应
            print("\n[测试步骤] 3. 生成响应")
            response = generate_response(recognized_text)
            print(f"[智能响应] {response}")
        else:
            print("\n[测试结果] ⚠ 未识别到语音内容")
            print(f"[录音文件] {audio_file} (已保存，可供分析)")
            
    except Exception as e:
        print(f"[测试异常] 单次测试过程异常: {e}")
    
    print("\n" + "=" * 60)
    print("[单次测试] 测试完成")
    print("=" * 60)

# ==================== 系统状态检查函数 ====================
def system_status_check():
    """
    系统状态检查
    """
    print("\n" + "=" * 60)
    print("[状态检查] 语音识别系统状态检查")
    print("=" * 60)
    
    # 检查Vosk模型
    print("\n[检查项目] === Vosk语音识别模型 ===")
    if model:
        print("[状态] ✓ Vosk模型已加载")
        print(f"[模型路径] {model_path}")
    else:
        print("[状态] ✗ Vosk模型未加载")
        print(f"[模型路径] {model_path}")
        if not os.path.exists(model_path):
            print("[问题] 模型文件不存在")
        
    # 检查USB麦克风
    print("\n[检查项目] === USB麦克风设备 ===")
    try:
        result = subprocess.run(['arecord', '-l'], capture_output=True, text=True)
        if result.returncode == 0:
            print("[状态] ✓ 录音设备可用")
            print(f"[设备] {usb_mic_device}")
        else:
            print(f"[状态] ✗ 录音设备检查失败: {result.stderr}")
    except Exception as e:
        print(f"[检查错误] 设备检查异常: {e}")
    
    # 检查音频卡
    print("\n[检查项目] === 音频卡信息 ===")
    try:
        with open('/proc/asound/cards', 'r') as f:
            cards_info = f.read().strip()
            if "UACDemoV10" in cards_info:
                print("[状态] ✓ USB音频设备已识别")
            print(f"[设备信息]\n{cards_info}")
    except Exception as e:
        print(f"[检查错误] 无法读取音频卡信息: {e}")
    
    print("=" * 60)

# ==================== 录音文件管理函数 ====================
def manage_audio_files():
    """
    管理录音文件
    """
    print("\n" + "=" * 60)
    print("[文件管理] 录音文件管理")
    print("=" * 60)
    
    # 查找所有录音文件
    audio_files = []
    for filename in os.listdir('.'):
        if filename.startswith('voice_record_') and filename.endswith('.wav'):
            audio_files.append(filename)
    
    if not audio_files:
        print("[文件状态] 没有找到录音文件")
        return
    
    # 按时间排序
    audio_files.sort()
    
    print(f"[文件状态] 找到 {len(audio_files)} 个录音文件:")
    total_size = 0
    
    for i, filename in enumerate(audio_files, 1):
        try:
            file_size = os.path.getsize(filename)
            total_size += file_size
            
            # 获取文件时间
            timestamp = int(filename.split('_')[2].split('.')[0])
            file_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(timestamp))
            
            print(f"  {i:2d}. {filename}")
            print(f"      时间: {file_time}")
            print(f"      大小: {file_size:,} bytes ({file_size/1024:.1f} KB)")
            
            # 尝试分析音频
            try:
                audio_info = analyze_audio_file(filename)
                if audio_info:
                    print(f"      时长: {audio_info['duration']:.1f} 秒")
            except:
                pass
            print()
            
        except Exception as e:
            print(f"  {i:2d}. {filename} - 错误: {e}")
    
    print(f"[统计信息] 总计: {len(audio_files)} 个文件, {total_size:,} bytes ({total_size/1024/1024:.1f} MB)")
    
    # 询问是否要清理旧文件
    print("\n[清理选项]")
    choice = input("是否要删除所有录音文件? (y/n): ").strip().lower()
    
    if choice == 'y':
        deleted_count = 0
        for filename in audio_files:
            try:
                os.remove(filename)
                deleted_count += 1
            except Exception as e:
                print(f"删除 {filename} 失败: {e}")
        print(f"[清理完成] 已删除 {deleted_count} 个文件")
    else:
        print("[清理取消] 文件保持不变")
    
    print("=" * 60)

# ==================== 主程序 ====================
if __name__ == "__main__":
    try:
        print("真实语音识别系统 V1.0")
        print("功能：USB麦克风录音 + Vosk语音识别 + 智能响应")
        print("说明：支持中文语音识别，录音文件自动保存")
        
        if not model:
            print("\n⚠️  警告：Vosk语音识别模型未加载！")
            print("请确保已下载并解压语音识别模型到当前目录")
            print("模型下载地址：https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip")
        
        while True:
            print("\n请选择操作:")
            print("1. 系统状态检查")
            print("2. 单次语音识别测试") 
            print("3. 开始语音识别会话")
            print("4. 录音文件管理")
            print("5. 退出程序")
            
            choice = input("请输入选择 (1-5): ").strip()
            
            if choice == '1':
                system_status_check()
            elif choice == '2':
                single_recognition_test()
            elif choice == '3':
                voice_recognition_session()
            elif choice == '4':
                manage_audio_files()
            elif choice == '5':
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