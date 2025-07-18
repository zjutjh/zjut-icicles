#!/usr/bin/env python
# -*- coding: utf-8 -*-

import requests
import base64

# API密钥
API_KEY = "AIzaSyA_bIYCfw1IGZoYn1RitnDcIby3Iv3qhM8"

# 预设声音列表 - 只保留两个高质量声音选项
VOICE_TYPES = {
    "1": {"name": "cmn-CN-Wavenet-C", "gender": "MALE", "description": "高质量男声 (Wavenet C)"},
    "2": {"name": "cmn-CN-Wavenet-D", "gender": "FEMALE", "description": "高质量女声 (Wavenet D)"}
}

def show_voice_options():
    """显示所有可用的声音选项"""
    print("\n可用的声音选项:")
    for key, voice in VOICE_TYPES.items():
        print(f"{key}. {voice['description']}")
    print()

def text_to_speech(text, output_file="output.mp3", language_code="cmn-CN", voice_type="1"):
    """
    使用Google TTS API将文本转换为语音并保存为音频文件
    
    参数:
        text: 要转换的文本
        output_file: 输出音频文件名
        language_code: 语言代码
        voice_type: 声音类型选项
    """
    # 获取选择的声音参数
    voice_option = VOICE_TYPES.get(voice_type, VOICE_TYPES["1"])
    
    # Google TTS API端点
    url = f"https://texttospeech.googleapis.com/v1/text:synthesize?key={API_KEY}"
    
    # 请求参数
    data = {
        "input": {
            "text": text
        },
        "voice": {
            "languageCode": language_code,
            "name": voice_option["name"],
            "ssmlGender": voice_option["gender"]
        },
        "audioConfig": {
            "audioEncoding": "MP3",
            "speakingRate": 1.0,  # 语速 (0.25-4.0)
            "pitch": 0.0  # 音调 (-20.0-20.0)
        }
    }
    
    print(f"正在使用 {voice_option['description']} 合成语音...")
    
    # 发送请求
    response = requests.post(url, json=data)
    
    if response.status_code == 200:
        # 解析响应
        audio_content = response.json().get("audioContent")
        
        # 将base64编码的音频内容解码为二进制
        audio_binary = base64.b64decode(audio_content)
        
        # 保存音频文件
        with open(output_file, "wb") as out:
            out.write(audio_binary)
            print(f"音频内容已保存到 {output_file}")
    else:
        print(f"请求失败，状态码: {response.status_code}")
        print(f"错误信息: {response.text}")

if __name__ == "__main__":
    # 显示声音选项
    show_voice_options()
    
    # 获取用户输入
    voice_choice = input(f"请选择声音类型 (1-男声, 2-女声，默认为1): ").strip() or "1"
    user_text = input("请输入要转换为语音的文本: ")
    
    # 生成语音
    text_to_speech(user_text, voice_type=voice_choice) 