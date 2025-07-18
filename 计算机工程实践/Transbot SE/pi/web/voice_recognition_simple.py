#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
语音识别程序 (简化代理版)
基于test.py的简洁实现 + 实时录音功能
"""

import speech_recognition as sr
import pyaudio
import base64
import requests
import json

# Google Cloud Speech API 配置
API_KEY = "AIzaSyDvNR2jkSe5gQfhwKLkCs1YBI-W_FefAwE"

# 代理配置
PROXIES = {
    'http': 'http://127.0.0.1:7890',
    'https': 'http://127.0.0.1:7890'
}

def get_usb_microphone():
    """获取USB麦克风设备"""
    audio = pyaudio.PyAudio()
    
    for i in range(audio.get_device_count()):
        try:
            device_info = audio.get_device_info_by_index(i)
            device_name = device_info['name']
            max_input_channels = device_info['maxInputChannels']
            
            if ('usb' in device_name.lower() or 'audio2.0' in device_name.lower() 
                or 'usbaudio' in device_name.lower()) and max_input_channels > 0:
                audio.terminate()
                return i
        except:
            continue
    
    audio.terminate()
    return None

def recognize_with_proxy(audio_data, language='zh-CN'):
    """使用代理调用Google Cloud Speech API - 采用test.py的简洁实现"""
    try:
        # 获取音频数据并转为base64 (完全按照test.py的方式)
        audio_content = base64.b64encode(audio_data.get_wav_data()).decode("utf-8")
        
        url = f"https://speech.googleapis.com/v1/speech:recognize?key={API_KEY}"
        headers = {"Content-Type": "application/json"}
        
        # 使用test.py的极简配置
        data = {
            "config": {
                "languageCode": language,
                "enableAutomaticPunctuation": True
            },
            "audio": {
                "content": audio_content
            }
        }
        
        # 发送请求 (完全按照test.py的方式)
        response = requests.post(url, headers=headers, data=json.dumps(data), proxies=PROXIES)
        result = response.json()
        
        # 简洁的结果处理
        if "results" in result:
            transcript = result["results"][0]["alternatives"][0]["transcript"]
            confidence = result["results"][0]["alternatives"][0].get("confidence", 0.0)
            return transcript, confidence
        else:
            print("识别失败:", result)
            return None, 0.0
            
    except requests.exceptions.ProxyError:
        print("❌ 代理连接失败")
        return None, 0.0
    except Exception as e:
        print(f"❌ 识别异常: {e}")
        return None, 0.0

def record_and_recognize(device_index=None, language='zh'):
    """录制并识别语音"""
    recognizer = sr.Recognizer()
    
    try:
        # 设置麦克风
        if device_index is not None:
            microphone = sr.Microphone(device_index=device_index)
        else:
            microphone = sr.Microphone()
        
        # 校准和录音
        with microphone as source:
            recognizer.adjust_for_ambient_noise(source, duration=1)
        
        print("🎤 开始录音，请说话...")
        with microphone as source:
            audio_data = recognizer.listen(source, timeout=5, phrase_time_limit=5)
        
        print("🧠 识别中...")
        
        # 设置语言
        lang_code = 'zh-CN' if language == 'zh' else 'en-US'
        
        # 优先使用代理版Google Cloud API
        text, confidence = recognize_with_proxy(audio_data, lang_code)
        
        if text:
            return text, confidence
        else:
            # 简单降级到免费版
            print("🔄 尝试免费版...")
            try:
                text = recognizer.recognize_google(audio_data, language=lang_code)
                return text, 0.0
            except:
                return None, 0.0
                
    except Exception as e:
        print(f"录音失败: {e}")
        return None, 0.0

def main():
    """主程序"""
    print("🎤 语音识别程序 (简化版)")
    print("=" * 30)
    print("💡 基于test.py的简洁实现")
    print("=" * 30)
    
    # 检测USB麦克风
    usb_device = get_usb_microphone()
    if usb_device:
        print(f"✅ 使用USB麦克风 (设备 {usb_device})")
    else:
        print("📱 使用默认麦克风")
    
    while True:
        print("\n" + "="*30)
        print("1. 按Enter - 中文识别")
        print("2. 输入'e' - 英文识别")
        print("3. 输入'q' - 退出")
        print("="*30)
        
        user_input = input("请选择: ").strip().lower()
        
        if user_input == 'q':
            print("👋 再见")
            break
        elif user_input == 'e':
            result, confidence = record_and_recognize(usb_device, language='en')
        else:
            result, confidence = record_and_recognize(usb_device, language='zh')
        
        if result:
            if confidence > 0:
                print(f"📝 识别结果: {result}")
                print(f"📊 置信度: {confidence:.2f}")
            else:
                print(f"📝 识别结果: {result}")
        else:
            print("❌ 未能识别")

if __name__ == "__main__":
    main() 