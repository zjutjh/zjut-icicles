import requests
import os

# --- 配置 ---
# 服务器地址
url = "http://localhost:5000/api/scan_and_upload_order"
# 要上传的图片路径 (相对于此脚本)
image_path = os.path.join("webapp", "微信二维码扫描模块", "0eeac6da486b2197ce07d1c069183e8.png")

# --- 函数 ---
def test_upload(file_path):
    """
    向服务器上传图片文件并打印响应结果
    """
    if not os.path.exists(file_path):
        print(f"❌ 文件不存在: {file_path}")
        return

    print(f"🚀 正在上传图片: {file_path}")
    
    with open(file_path, 'rb') as f:
        files = {'file': (os.path.basename(file_path), f, 'image/jpeg')}
        try:
            response = requests.post(url, files=files, timeout=30)
            
            # 检查响应状态码
            if response.status_code == 200:
                print("✅ 请求成功!")
                print("服务器响应:")
                print(response.json())
            else:
                print(f"❌ 请求失败，状态码: {response.status_code}")
                print("服务器响应:")
                print(response.text)

        except requests.exceptions.RequestException as e:
            print(f"💥 请求出错: {e}")

# --- 主程序 ---
if __name__ == "__main__":
    test_upload(image_path) 