#!/usr/bin/env python3
import cv2

def detect_qrcode(image_path):
    """检测并解码图片中的二维码"""
    # 读取图片
    img = cv2.imread(image_path)
    if img is None:
        print(f"无法读取图片: {image_path}")
        return
    
    # 创建QR码检测器
    qcd = cv2.QRCodeDetector()
    
    # 检测并解码二维码
    retval, decoded_info, points, straight_qrcode = qcd.detectAndDecodeMulti(img)
    
    if retval:
        print(f"检测到 {len(decoded_info)} 个二维码:")
        for i, info in enumerate(decoded_info):
            if info:
                print(f"二维码 {i+1}: {info}")
            else:
                print(f"二维码 {i+1}: 检测到但无法解码")
        
        # 在图片上绘制检测框
        for i, p in enumerate(points):
            color = (0, 255, 0) if decoded_info[i] else (0, 0, 255)
            img = cv2.polylines(img, [p.astype(int)], True, color, 3)
        
        # 保存标注后的图片
        cv2.imwrite('result_qrcode.png', img)
        print("结果已保存到 result_qrcode.png")
    else:
        print("未检测到二维码")

if __name__ == "__main__":
    detect_qrcode("test.png")
