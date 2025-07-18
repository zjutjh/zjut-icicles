import cv2
import sys
import os
from pyzbar.pyzbar import decode
from PIL import Image

# 获取脚本所在目录，并构建模型文件的绝对路径
script_dir = os.path.dirname(os.path.abspath(__file__))
detect_prototxt_path = os.path.join(script_dir, "detect.prototxt")
detect_caffemodel_path = os.path.join(script_dir, "detect.caffemodel")
sr_prototxt_path = os.path.join(script_dir, "sr.prototxt")
sr_caffemodel_path = os.path.join(script_dir, "sr.caffemodel")

def scan_qrcode(image_path):
    """
    使用微信开源引擎扫描二维码，如果失败则尝试使用pyzbar
    :param image_path: 图片路径
    :return: 识别结果列表
    """
    # 优先使用微信引擎
    try:
        detector = cv2.wechat_qrcode.WeChatQRCode(
            detect_prototxt_path,
            detect_caffemodel_path,
            sr_prototxt_path,
            sr_caffemodel_path
        )
        img = cv2.imread(image_path)
        if img is not None:
            # 图像增强
            img = cv2.convertScaleAbs(img, alpha=1.5, beta=50)
            results, _ = detector.detectAndDecode(img)
            # 过滤有效结果
            valid_results = [res for res in results if res]
            if valid_results:
                print("✅ 使用微信引擎识别成功")
                return valid_results
    except Exception as e:
        print(f"⚠️ 微信引擎初始化或扫描失败: {e}")

    # 如果微信引擎失败，尝试使用pyzbar
    print("ℹ️ 微信引擎识别失败，正在尝试使用 pyzbar...")
    try:
        img_pil = Image.open(image_path)
        decoded_objects = decode(img_pil)
        if decoded_objects:
            results = [obj.data.decode('utf-8') for obj in decoded_objects if obj.data]
            print("✅ 使用 pyzbar 识别成功")
            return results
    except Exception as e:
        print(f"❌ pyzbar 识别失败: {e}")

    return [] # 如果都失败，返回空列表

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python wechat_qrcode_scanner.py <image_path>")
        sys.exit(1)
    
    image_path = sys.argv[1]
    qr_results = scan_qrcode(image_path)
    
    if qr_results:
        print("✅ 识别结果：")
        for i, res in enumerate(qr_results):
            print(f"  {i+1}. {res}")
    else:
        print("❌ 未检测到二维码")