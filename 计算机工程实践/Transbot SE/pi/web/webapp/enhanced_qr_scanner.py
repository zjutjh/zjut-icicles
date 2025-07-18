import cv2
import numpy as np
from pyzbar.pyzbar import decode
from PIL import Image, ImageEnhance
import os
import logging

# 导入qreader - 基于深度学习的二维码识别库
try:
    from qreader import QReader
    QREADER_AVAILABLE = True
    logging.info("✅ qreader模块导入成功")
except ImportError as e:
    QREADER_AVAILABLE = False
    logging.warning(f"⚠️ qreader模块导入失败: {e}")

# 全局qreader实例
qreader_detector = None
if QREADER_AVAILABLE:
    try:
        qreader_detector = QReader()
        logging.info("✅ QReader检测器初始化成功")
    except Exception as e:
        logging.error(f"❌ QReader检测器初始化失败: {e}")
        QREADER_AVAILABLE = False

def preprocess_image(image):
    """
    对图像进行预处理以提高二维码识别率
    包括多种增强技术
    """
    processed_images = []
    
    # 确保图像数据类型正确
    if image.dtype != np.uint8:
        image = image.astype(np.uint8)
    
    # 原始图像
    processed_images.append(("原始", image))
    
    # 转为灰度图
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()
    
    # 确保灰度图数据类型正确
    if gray.dtype != np.uint8:
        gray = gray.astype(np.uint8)
    
    processed_images.append(("灰度", gray))
    
    # 应用高斯模糊减少噪声
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    processed_images.append(("模糊", blurred))
    
    # 直方图均衡化
    equalized = cv2.equalizeHist(gray)
    processed_images.append(("均衡化", equalized))
    
    # 自适应阈值处理
    adaptive_thresh = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
    )
    processed_images.append(("自适应阈值", adaptive_thresh))
    
    # OTSU二值化
    _, otsu_thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    processed_images.append(("OTSU阈值", otsu_thresh))
    
    # 形态学操作 - 开运算
    kernel = np.ones((3, 3), np.uint8)
    opened = cv2.morphologyEx(adaptive_thresh, cv2.MORPH_OPEN, kernel)
    processed_images.append(("开运算", opened))
    
    # 边缘增强
    sharpening_kernel = np.array([[-1,-1,-1],
                                  [-1, 9,-1],
                                  [-1,-1,-1]])
    sharpened = cv2.filter2D(gray, -1, sharpening_kernel)
    processed_images.append(("锐化", sharpened))
    
    # 对比度增强
    enhanced = cv2.convertScaleAbs(gray, alpha=1.5, beta=30)
    processed_images.append(("对比度增强", enhanced))
    
    # Gamma校正 - 暗图像
    gamma_dark = 0.5
    table_dark = np.array([((i / 255.0) ** gamma_dark) * 255 for i in np.arange(0, 256)]).astype("uint8")
    gamma_corrected_dark = cv2.LUT(gray, table_dark)
    processed_images.append(("Gamma暗校正", gamma_corrected_dark))
    
    # Gamma校正 - 亮图像
    gamma_bright = 2.0
    table_bright = np.array([((i / 255.0) ** gamma_bright) * 255 for i in np.arange(0, 256)]).astype("uint8")
    gamma_corrected_bright = cv2.LUT(gray, table_bright)
    processed_images.append(("Gamma亮校正", gamma_corrected_bright))
    
    return processed_images

def resize_image_variants(image):
    """
    生成不同尺寸的图像变体
    """
    variants = []
    height, width = image.shape[:2]
    
    # 原始尺寸
    variants.append(("原始尺寸", image))
    
    # 放大版本
    for scale in [1.5, 2.0]:
        new_width = int(width * scale)
        new_height = int(height * scale)
        resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_CUBIC)
        variants.append((f"放大{scale}x", resized))
    
    # 缩小版本
    for scale in [0.8, 0.5]:
        new_width = int(width * scale)
        new_height = int(height * scale)
        if new_width > 100 and new_height > 100:  # 确保图像不会太小
            resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)
            variants.append((f"缩小{scale}x", resized))
    
    return variants

def rotate_image_variants(image):
    """
    生成不同旋转角度的图像变体
    """
    variants = []
    height, width = image.shape[:2]
    center = (width // 2, height // 2)
    
    # 原始角度
    variants.append(("原始角度", image))
    
    # 小角度旋转
    for angle in [-15, -10, -5, 5, 10, 15]:
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated = cv2.warpAffine(image, rotation_matrix, (width, height), 
                                flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
        variants.append((f"旋转{angle}度", rotated))
    
    # 90度旋转
    for angle in [90, 180, 270]:
        if angle == 90:
            rotated = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        elif angle == 180:
            rotated = cv2.rotate(image, cv2.ROTATE_180)
        elif angle == 270:
            rotated = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        variants.append((f"旋转{angle}度", rotated))
    
    return variants

def decode_qr_with_qreader(image, method_name=""):
    """
    使用qreader解码二维码 - 基于深度学习，对复杂背景识别效果更好
    """
    if not QREADER_AVAILABLE or qreader_detector is None:
        return []
    
    try:
        # qreader需要RGB格式的图像
        if len(image.shape) == 3:
            # 如果是BGR格式，转换为RGB
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        else:
            # 灰度图转RGB
            rgb_image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        
        # 使用qreader检测和解码
        decoded_texts = qreader_detector.detect_and_decode(image=rgb_image)
        
        if decoded_texts:
            # 过滤空字符串和None值
            results = [text for text in decoded_texts if text and text.strip()]
            if results and method_name:
                logging.info(f"✅ qreader方法 '{method_name}' 识别成功: {results}")
            return results
    except Exception as e:
        if method_name:
            logging.debug(f"qreader方法 '{method_name}' 识别失败: {e}")
    return []

def decode_qr_from_image(image, method_name=""):
    """
    从图像中解码二维码 - 使用pyzbar作为备用方案
    """
    try:
        # 使用pyzbar解码
        decoded_objects = decode(image)
        if decoded_objects:
            results = []
            for obj in decoded_objects:
                try:
                    text = obj.data.decode('utf-8')
                    if text.strip():  # 确保不是空字符串
                        results.append(text)
                        if method_name:
                            logging.info(f"✅ pyzbar方法 '{method_name}' 识别成功: {text}")
                except UnicodeDecodeError:
                    # 尝试其他编码
                    try:
                        text = obj.data.decode('gbk')
                        if text.strip():
                            results.append(text)
                            if method_name:
                                logging.info(f"✅ pyzbar方法 '{method_name}' (GBK编码) 识别成功: {text}")
                    except:
                        continue
            return results
    except Exception as e:
        if method_name:
            logging.debug(f"pyzbar方法 '{method_name}' 识别失败: {e}")
    return []

def scan_qrcode(image_path):
    """
    增强版二维码扫描函数
    使用多种图像处理技术提高识别率
    :param image_path: 图片路径
    :return: 识别结果列表
    """
    if not os.path.exists(image_path):
        logging.error(f"图片文件不存在: {image_path}")
        return []
    
    try:
        # 使用numpy和cv2来处理中文路径问题
        # 先用PIL读取，再转换为opencv格式
        from PIL import Image as PILImage
        pil_image = PILImage.open(image_path)
        
        # 转换为numpy数组
        image_array = np.array(pil_image)
        
        # 确保数据类型为uint8
        if image_array.dtype != np.uint8:
            # 如果是浮点型，需要转换范围
            if image_array.dtype in [np.float32, np.float64]:
                image_array = (image_array * 255).astype(np.uint8)
            else:
                image_array = image_array.astype(np.uint8)
        
        # 如果是RGB格式，转换为BGR（OpenCV格式）
        if len(image_array.shape) == 3 and image_array.shape[2] == 3:
            image = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
        elif len(image_array.shape) == 3 and image_array.shape[2] == 4:
            # RGBA格式，转换为BGR
            image = cv2.cvtColor(image_array, cv2.COLOR_RGBA2BGR)
        else:
            # 灰度图像
            image = image_array
        
        if image is None:
            logging.error(f"无法读取图片: {image_path}")
            return []
        
        logging.info(f"开始增强识别图片: {image_path}")
        logging.info(f"图片尺寸: {image.shape}")
        
        all_results = set()  # 使用set避免重复结果
        
        # 第一轮：优先使用qreader识别原图（基于深度学习，对复杂背景效果更好）
        if QREADER_AVAILABLE:
            logging.info("🔍 使用qreader进行深度学习识别...")
            results = decode_qr_with_qreader(image, "qreader-原始图像")
            all_results.update(results)
            
            # 如果qreader成功识别，立即返回结果
            if all_results:
                logging.info(f"🎉 qreader识别成功，结果: {list(all_results)}")
                return list(all_results)
        
        # 第二轮：如果qreader失败，尝试传统方法识别原图
        logging.info("🔄 qreader未找到结果，尝试传统pyzbar方法...")
        results = decode_qr_from_image(image, "pyzbar-原始图像")
        all_results.update(results)
        
        # 如果传统方法成功识别，可以提前返回
        if all_results:
            logging.info(f"传统方法识别成功，结果: {list(all_results)}")
            return list(all_results)
        
        # 第三轮：对预处理后的图像使用qreader
        if QREADER_AVAILABLE and not all_results:
            logging.info("🔍 对预处理图像使用qreader识别...")
            processed_images = preprocess_image(image)
            for method_name, processed_img in processed_images:
                results = decode_qr_with_qreader(processed_img, f"qreader-预处理-{method_name}")
                all_results.update(results)
                
                # 如果qreader在预处理图像上找到结果，可以提前返回
                if results:
                    logging.info(f"🎉 qreader在预处理图像 '{method_name}' 上找到结果: {results}")
                    break  # qreader找到结果就停止，避免过度处理
        
        # 第四轮：如果qreader仍未找到结果，使用传统方法处理预处理图像
        if not all_results:
            logging.info("🔄 使用传统方法处理预处理图像...")
            processed_images = preprocess_image(image)
            for method_name, processed_img in processed_images:
                results = decode_qr_from_image(processed_img, f"pyzbar-预处理-{method_name}")
                all_results.update(results)
                
                # 如果找到结果，继续尝试其他方法以找到更多可能的二维码
                if results:
                    logging.info(f"传统预处理方法 '{method_name}' 找到结果: {results}")
        
        # 第五轮：尝试不同尺寸（仅在前面方法都失败时）
        if not all_results:
            logging.info("🔄 尝试不同尺寸变体...")
            resize_variants = resize_image_variants(image)
            for variant_name, variant_img in resize_variants:
                # 优先尝试qreader
                if QREADER_AVAILABLE:
                    results = decode_qr_with_qreader(variant_img, f"qreader-尺寸变体-{variant_name}")
                    all_results.update(results)
                    if results:
                        logging.info(f"🎉 qreader尺寸变体 '{variant_name}' 找到结果: {results}")
                        break
                
                # 如果qreader失败，尝试传统方法
                results = decode_qr_from_image(variant_img, f"pyzbar-尺寸变体-{variant_name}")
                all_results.update(results)
                if results:
                    logging.info(f"传统尺寸变体 '{variant_name}' 找到结果: {results}")
                    break
        
        # 第六轮：尝试旋转图像（仅在前面方法都失败时）
        if not all_results:
            logging.info("🔄 尝试旋转变体...")
            # 使用灰度图进行旋转以提高效率
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
            rotation_variants = rotate_image_variants(gray_image)
            for variant_name, variant_img in rotation_variants:
                # 优先尝试qreader
                if QREADER_AVAILABLE:
                    results = decode_qr_with_qreader(variant_img, f"qreader-旋转变体-{variant_name}")
                    all_results.update(results)
                    if results:
                        logging.info(f"🎉 qreader旋转变体 '{variant_name}' 找到结果: {results}")
                        break
                
                # 如果qreader失败，尝试传统方法
                results = decode_qr_from_image(variant_img, f"pyzbar-旋转变体-{variant_name}")
                all_results.update(results)
                if results:
                    logging.info(f"传统旋转变体 '{variant_name}' 找到结果: {results}")
                    break
        
        final_results = list(all_results)
        if final_results:
            logging.info(f"✅ 最终识别成功，共找到 {len(final_results)} 个二维码: {final_results}")
        else:
            logging.warning(f"❌ 所有方法都无法识别二维码: {image_path}")
        
        return final_results
        
    except Exception as e:
        logging.error(f"二维码识别过程中出错: {str(e)}")
        return []

# 兼容性函数 - 保持与原模块相同的接口
def main():
    """测试函数"""
    import sys
    if len(sys.argv) < 2:
        print("Usage: python enhanced_qr_scanner.py <image_path>")
        sys.exit(1)
    
    image_path = sys.argv[1]
    results = scan_qrcode(image_path)
    
    if results:
        print("✅ 识别结果：")
        for i, result in enumerate(results):
            print(f"  {i+1}. {result}")
    else:
        print("❌ 未检测到二维码")

if __name__ == "__main__":
    # 设置日志级别
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    main() 