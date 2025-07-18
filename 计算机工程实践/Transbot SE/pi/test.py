import base64
import requests
import json

API_KEY = "AIzaSyDvNR2jkSe5gQfhwKLkCs1YBI-W_FefAwE"

# 读取音频文件并转为 base64
with open("t1.wav", "rb") as audio_file:
    audio_content = base64.b64encode(audio_file.read()).decode("utf-8")

url = f"https://speech.googleapis.com/v1/speech:recognize?key={API_KEY}"

headers = {"Content-Type": "application/json"}

data = {
    "config": {
        "languageCode": "zh-CN",
        "enableAutomaticPunctuation": True  # 自动添加标点符号
    },
    "audio": {
        "content": audio_content
    }
}

# 配置 Clash 代理（默认端口7890，如果不同请修改）
proxies = {
    'http': 'http://127.0.0.1:7890',
    'https': 'http://127.0.0.1:7890'
}

# 使用代理发送请求
response = requests.post(url, headers=headers, data=json.dumps(data), proxies=proxies)
result = response.json()
print(json.dumps(result, ensure_ascii=False, indent=2))

# 输出识别文本
if "results" in result:
    for r in result["results"]:
        print("识别结果:", r["alternatives"][0]["transcript"])
else:
    print("识别失败:", result)