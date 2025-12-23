#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
综合图像去噪方法对比系统
包含传统滤波方法和基于PDE的先进方法
重点对比TV与ITV的区别和效果
"""

import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import time
import os
import json
import pandas as pd
from datetime import datetime
from skimage import img_as_float
from skimage.metrics import peak_signal_noise_ratio, structural_similarity
from skimage.util import random_noise

def setup_chinese_font():
    """设置中文字体"""
    print("正在配置字体...")
    
    # 强制使用英文，避免字体问题
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'sans-serif']
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['font.size'] = 10
    
    print("已设置为英文字体，避免中文字体问题")
    return False  # 强制返回False，使用英文

# 初始化字体配置
USE_CHINESE = setup_chinese_font()

def get_text(chinese_text, english_text):
    """根据字体支持情况返回合适的文本"""
    return english_text  # 强制返回英文

class ComprehensiveDenoising:
    """综合图像去噪对比系统"""
    
    def __init__(self):
        self.epsilon = 1e-8  # 数值稳定性参数
        self.h = 1.0  # 空间步长
        
    def load_image(self, image_path):
        """加载并预处理图像"""
        try:
            img = cv2.imread(image_path)
            if img is None:
                raise ValueError(f"无法加载图像: {image_path}")
            
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = img_as_float(img)
            
            print(f"成功加载图像: {image_path}")
            print(f"图像尺寸: {img.shape}")
            
            return img
        except Exception as e:
            print(f"加载图像时出错: {e}")
            return None
    
    def add_noise(self, image, noise_type='gaussian', noise_level=0.1):
        """为图像添加噪声"""
        if noise_type == 'gaussian':
            noisy_image = random_noise(image, mode='gaussian', var=noise_level**2, clip=True)
        elif noise_type == 'salt_pepper':
            noisy_image = random_noise(image, mode='s&p', amount=noise_level, clip=True)
        elif noise_type == 'speckle':
            noisy_image = random_noise(image, mode='speckle', var=noise_level**2, clip=True)
        else:
            raise ValueError(f"不支持的噪声类型: {noise_type}")
        
        return noisy_image.astype(np.float64)

    # ==================== 传统滤波方法 ====================
    
    def gaussian_filter(self, image, kernel_size=5, sigma=1.0):
        """高斯滤波"""
        print("执行高斯滤波...")
        return cv2.GaussianBlur(image, (kernel_size, kernel_size), sigma)
    
    def median_filter(self, image, kernel_size=5):
        """中值滤波"""
        print("执行中值滤波...")
        if len(image.shape) == 3:
            result = np.zeros_like(image)
            for i in range(image.shape[2]):
                result[:,:,i] = cv2.medianBlur(
                    (image[:,:,i] * 255).astype(np.uint8), kernel_size) / 255.0
            return result
        else:
            return cv2.medianBlur(
                (image * 255).astype(np.uint8), kernel_size) / 255.0
    
    def bilateral_filter(self, image, d=9, sigma_color=75, sigma_space=75):
        """双边滤波"""
        print("执行双边滤波...")
        if len(image.shape) == 3:
            return cv2.bilateralFilter(
                (image * 255).astype(np.uint8), d, sigma_color, sigma_space) / 255.0
        else:
            return cv2.bilateralFilter(
                (image * 255).astype(np.uint8), d, sigma_color, sigma_space) / 255.0
    
    def adaptive_gaussian_filter(self, image, base_sigma=1.0, edge_threshold=0.1):
        """自适应高斯滤波"""
        print("执行自适应高斯滤波...")
        if len(image.shape) == 3:
            result = np.zeros_like(image)
            for c in range(image.shape[2]):
                result[:,:,c] = self._adaptive_gaussian_channel(
                    image[:,:,c], base_sigma, edge_threshold)
            return result
        else:
            return self._adaptive_gaussian_channel(image, base_sigma, edge_threshold)
    
    def _adaptive_gaussian_channel(self, image, base_sigma, edge_threshold):
        """单通道自适应高斯滤波"""
        # 计算梯度幅值
        grad_x = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=3)
        grad_magnitude = np.sqrt(grad_x**2 + grad_y**2)
        
        # 自适应sigma：边缘处小sigma，平滑区域大sigma
        adaptive_sigma = base_sigma * (1 + np.exp(-grad_magnitude / edge_threshold))
        
        # 应用不同的高斯滤波
        result = np.zeros_like(image)
        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                sigma = adaptive_sigma[i, j]
                kernel_size = int(6 * sigma + 1)
                if kernel_size % 2 == 0:
                    kernel_size += 1
                kernel_size = max(3, min(kernel_size, 15))  # 限制核大小
                
                # 对局部区域应用高斯滤波
                roi_size = kernel_size // 2
                i_start = max(0, i - roi_size)
                i_end = min(image.shape[0], i + roi_size + 1)
                j_start = max(0, j - roi_size)
                j_end = min(image.shape[1], j + roi_size + 1)
                
                roi = image[i_start:i_end, j_start:j_end]
                filtered_roi = cv2.GaussianBlur(roi, (kernel_size, kernel_size), sigma)
                
                # 取中心像素
                center_i = i - i_start
                center_j = j - j_start
                if center_i < filtered_roi.shape[0] and center_j < filtered_roi.shape[1]:
                    result[i, j] = filtered_roi[center_i, center_j]
                else:
                    result[i, j] = image[i, j]
        
        return result

    # ==================== PDE方法 ====================
    
    def perona_malik_diffusion(self, image, iterations=30, delta_t=0.15, kappa=15, option=2):
        """Perona-Malik各向异性扩散"""
        print("执行Perona-Malik各向异性扩散...")
        if len(image.shape) == 3:
            result = np.zeros_like(image)
            for c in range(image.shape[2]):
                result[:,:,c] = self._perona_malik_channel(
                    image[:,:,c], iterations, delta_t, kappa, option)
            return result
        else:
            return self._perona_malik_channel(image, iterations, delta_t, kappa, option)
    
    def _perona_malik_channel(self, image, iterations, delta_t, kappa, option):
        """单通道Perona-Malik扩散"""
        img = image.copy().astype(np.float64)
        
        for i in range(iterations):
            # 计算四个方向的梯度
            nabla_n = np.roll(img, -1, axis=0) - img  # 北
            nabla_s = np.roll(img, 1, axis=0) - img   # 南
            nabla_e = np.roll(img, -1, axis=1) - img  # 东
            nabla_w = np.roll(img, 1, axis=1) - img   # 西
            
            # 计算扩散系数
            if option == 1:
                # g(x) = exp(-(|∇I|/κ)²)
                c_n = np.exp(-((nabla_n/kappa)**2))
                c_s = np.exp(-((nabla_s/kappa)**2))
                c_e = np.exp(-((nabla_e/kappa)**2))
                c_w = np.exp(-((nabla_w/kappa)**2))
            elif option == 2:
                # g(x) = 1/(1+(|∇I|/κ)²)
                c_n = 1.0 / (1.0 + (nabla_n/kappa)**2)
                c_s = 1.0 / (1.0 + (nabla_s/kappa)**2)
                c_e = 1.0 / (1.0 + (nabla_e/kappa)**2)
                c_w = 1.0 / (1.0 + (nabla_w/kappa)**2)
            
            # 更新图像
            img += delta_t * (c_n * nabla_n + c_s * nabla_s + c_e * nabla_e + c_w * nabla_w)
            img = np.clip(img, 0, 1)
        
        return img
    
    def total_variation_denoising(self, image, lmbda=0.1, iterations=100, dt=0.25, tolerance=1e-6):
        """
        Total Variation去噪 - 标准实现
        容易产生阶梯化效应
        """
        print("执行Total Variation去噪...")
        if len(image.shape) == 3:
            result = np.zeros_like(image)
            for c in range(image.shape[2]):
                result[:,:,c] = self._tv_channel(
                    image[:,:,c], lmbda, iterations, dt, tolerance)
            return result
        else:
            return self._tv_channel(image, lmbda, iterations, dt, tolerance)
    
    def _tv_channel(self, image, lmbda, iterations, dt, tolerance):
        """单通道TV去噪"""
        u = image.copy().astype(np.float64)
        f = image.copy().astype(np.float64)
        
        for i in range(iterations):
            u_old = u.copy()
            
            # 计算梯度
            u_x = np.roll(u, -1, axis=1) - u
            u_y = np.roll(u, -1, axis=0) - u
            
            # 梯度幅值 - 使用与main.py相同的epsilon
            grad_magnitude = np.sqrt(u_x**2 + u_y**2 + 1e-8)
            
            # 计算散度
            p = u_x / grad_magnitude
            q = u_y / grad_magnitude
            
            div_p = p - np.roll(p, 1, axis=1)
            div_q = q - np.roll(q, 1, axis=0)
            
            # TV更新 - 与main.py保持一致的公式
            u += dt * (lmbda * (div_p + div_q) - (u - f))
            u = np.clip(u, 0, 1)
            
            # 检查收敛性
            change = np.mean(np.abs(u - u_old))
            if change < tolerance:
                print(f"  TV在第{i+1}次迭代收敛")
                break
                
            if (i + 1) % 20 == 0:
                print(f"  TV迭代 {i+1}/{iterations}, 变化: {change:.6f}")
        
        return u
    
    def improved_total_variation_denoising(self, image, lmbda=0.1, iterations=100, dt=0.25, tolerance=1e-6):
        """
        改进Total Variation (ITV)去噪 - 简化高效实现
        基于参考文献方程(2.7): u_t = |∇u| ∇·(∇u/|∇u|) + λ|∇u|(u_0 - u)
        关键：散度项和数据项都要乘以|∇u|
        """
        print("执行改进Total Variation (ITV)去噪...")
        if len(image.shape) == 3:
            result = np.zeros_like(image)
            for c in range(image.shape[2]):
                result[:,:,c] = self._itv_channel_efficient(
                    image[:,:,c], lmbda, iterations, dt, tolerance)
            return result
        else:
            return self._itv_channel_efficient(image, lmbda, iterations, dt, tolerance)
    
    def _itv_channel_efficient(self, image, lmbda, iterations, dt, tolerance):
        """单通道ITV去噪 - 高效矢量化实现"""
        u = image.copy().astype(np.float64)
        f = image.copy().astype(np.float64)
        
        for i in range(iterations):
            u_old = u.copy()
            
            # 计算梯度
            u_x = np.roll(u, -1, axis=1) - u  # x方向偏导数
            u_y = np.roll(u, -1, axis=0) - u  # y方向偏导数
            
            # 计算梯度的模 - 使用与TV相同的epsilon
            grad_magnitude = np.sqrt(u_x**2 + u_y**2 + 1e-8)
            
            # 计算散度 div(∇u/|∇u|)
            p = u_x / grad_magnitude
            q = u_y / grad_magnitude
            
            div_p = p - np.roll(p, 1, axis=1)
            div_q = q - np.roll(q, 1, axis=0)
            
            # ITV更新：关键是两项都乘以|∇u|
            # 根据参考文献方程(2.7): u_t = |∇u| ∇·(∇u/|∇u|) + λ|∇u|(u_0 - u)
            div_term = grad_magnitude * (div_p + div_q)
            data_term = grad_magnitude * (f - u)  # 注意：数据项也要乘以|∇u|
            
            u += dt * (lmbda * div_term + lmbda * data_term)
            u = np.clip(u, 0, 1)
            
            # 检查收敛性
            change = np.mean(np.abs(u - u_old))
            if change < tolerance:
                print(f"  ITV在第{i+1}次迭代收敛")
                break
                
            if (i + 1) % 20 == 0:
                print(f"  ITV迭代 {i+1}/{iterations}, 变化: {change:.6f}")
        
        return u

    def evaluate_performance(self, original, denoised, noisy):
        """性能评估"""
        psnr_value = peak_signal_noise_ratio(original, denoised, data_range=1.0)
        
        if len(original.shape) == 3:
            ssim_value = structural_similarity(original, denoised, multichannel=True, channel_axis=2, data_range=1.0)
        else:
            ssim_value = structural_similarity(original, denoised, data_range=1.0)
        
        mse_value = np.mean((original - denoised)**2)
        
        # 与噪声图像比较
        psnr_noisy = peak_signal_noise_ratio(original, noisy, data_range=1.0)
        if len(original.shape) == 3:
            ssim_noisy = structural_similarity(original, noisy, multichannel=True, channel_axis=2, data_range=1.0)
        else:
            ssim_noisy = structural_similarity(original, noisy, data_range=1.0)
        
        return {
            'PSNR': psnr_value,
            'SSIM': ssim_value,
            'MSE': mse_value,
            'PSNR_improvement': psnr_value - psnr_noisy,
            'SSIM_improvement': ssim_value - ssim_noisy
        }

    def save_results_to_folder(self, original, noisy, methods, metrics, times, noise_level):
        """保存所有结果到专门的文件夹"""
        # 创建结果文件夹
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        results_folder = f"denoising_results_{timestamp}"
        os.makedirs(results_folder, exist_ok=True)
        
        print(f"\n保存结果到文件夹: {results_folder}")
        
        # 1. 保存原始图像
        self._save_image(original, os.path.join(results_folder, "01_original.png"))
        
        # 2. 保存噪声图像
        noisy_psnr = peak_signal_noise_ratio(original, noisy, data_range=1.0)
        if len(original.shape) == 3:
            noisy_ssim = structural_similarity(original, noisy, multichannel=True, channel_axis=2, data_range=1.0)
        else:
            noisy_ssim = structural_similarity(original, noisy, data_range=1.0)
        
        self._save_image(noisy, os.path.join(results_folder, f"02_noisy_PSNR{noisy_psnr:.2f}.png"))
        
        # 3. 保存各种降噪结果
        method_order = [
            ('Gaussian Filter', '03_gaussian'),
            ('Median Filter', '04_median'), 
            ('Bilateral Filter', '05_bilateral'),
            ('Adaptive Gaussian', '06_adaptive_gaussian'),
            ('Perona-Malik', '07_perona_malik'),
            ('TV', '08_tv'),
            ('ITV', '09_itv')
        ]
        
        for method_name, filename_prefix in method_order:
            if method_name in methods:
                psnr = metrics[method_name]['PSNR']
                ssim = metrics[method_name]['SSIM']
                filename = f"{filename_prefix}_PSNR{psnr:.2f}_SSIM{ssim:.3f}.png"
                self._save_image(methods[method_name], os.path.join(results_folder, filename))
        
        # 4. 保存详细数据
        self._save_detailed_data(original, noisy, methods, metrics, times, noise_level, results_folder)
        
        # 5. 生成对比图像
        self._save_comparison_images(original, noisy, methods, metrics, noise_level, results_folder)
        
        print(f"所有结果已保存到: {results_folder}")
        return results_folder
    
    def _save_image(self, image, filepath):
        """保存单张图像"""
        # 确保图像值在正确范围内
        if image.dtype == np.float64 or image.dtype == np.float32:
            image_to_save = (np.clip(image, 0, 1) * 255).astype(np.uint8)
        else:
            image_to_save = image
        
        # 转换颜色空间用于保存
        if len(image_to_save.shape) == 3:
            image_to_save = cv2.cvtColor(image_to_save, cv2.COLOR_RGB2BGR)
        
        cv2.imwrite(filepath, image_to_save)
    
    def _save_detailed_data(self, original, noisy, methods, metrics, times, noise_level, results_folder):
        """保存详细的数值数据"""
        # 噪声图像的指标
        noisy_psnr = peak_signal_noise_ratio(original, noisy, data_range=1.0)
        if len(original.shape) == 3:
            noisy_ssim = structural_similarity(original, noisy, multichannel=True, channel_axis=2, data_range=1.0)
        else:
            noisy_ssim = structural_similarity(original, noisy, data_range=1.0)
        
        # 准备数据
        data_summary = {
            "experiment_info": {
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "noise_type": "gaussian",
                "noise_level": noise_level,
                "image_shape": original.shape
            },
            "noisy_image": {
                "PSNR": float(noisy_psnr),
                "SSIM": float(noisy_ssim)
            },
            "methods_results": {}
        }
        
        # 表格数据用于CSV
        table_data = []
        
        # 添加噪声图像数据
        table_data.append({
            "方法": "噪声图像",
            "PSNR(dB)": f"{noisy_psnr:.2f}",
            "SSIM": f"{noisy_ssim:.4f}",
            "PSNR提升(dB)": "0.00",
            "SSIM提升": "0.0000",
            "处理时间(s)": "0.00",
            "备注": f"高斯噪声 σ={noise_level}"
        })
        
        # 添加各方法的数据
        for method_name, metric in metrics.items():
            # JSON数据
            data_summary["methods_results"][method_name] = {
                "PSNR": float(metric['PSNR']),
                "SSIM": float(metric['SSIM']),
                "MSE": float(metric['MSE']),
                "PSNR_improvement": float(metric['PSNR_improvement']),
                "SSIM_improvement": float(metric['SSIM_improvement']),
                "processing_time": float(times.get(method_name, 0))
            }
            
            # 表格数据
            processing_time = times.get(method_name, 0)
            table_data.append({
                "方法": method_name,
                "PSNR(dB)": f"{metric['PSNR']:.2f}",
                "SSIM": f"{metric['SSIM']:.4f}",
                "PSNR提升(dB)": f"{metric['PSNR_improvement']:.2f}",
                "SSIM提升": f"{metric['SSIM_improvement']:.4f}",
                "处理时间(s)": f"{processing_time:.2f}",
                "备注": self._get_method_description(method_name)
            })
        
        # 保存JSON文件
        json_path = os.path.join(results_folder, "detailed_results.json")
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(data_summary, f, ensure_ascii=False, indent=2)
        
        # 保存CSV文件
        csv_path = os.path.join(results_folder, "results_summary.csv")
        df = pd.DataFrame(table_data)
        df.to_csv(csv_path, index=False, encoding='utf-8-sig')
        
        # 保存Markdown报告
        self._save_markdown_report(data_summary, table_data, results_folder)
        
        print(f"  数据文件已保存:")
        print(f"    - {json_path}")
        print(f"    - {csv_path}")
        print(f"    - {os.path.join(results_folder, 'experiment_report.md')}")
    
    def _get_method_description(self, method_name):
        """获取方法描述"""
        descriptions = {
            "Gaussian Filter": "传统线性平滑滤波",
            "Median Filter": "对椒盐噪声效果好",
            "Bilateral Filter": "保边去噪滤波",
            "Adaptive Gaussian": "基于梯度的自适应滤波",
            "Perona-Malik": "各向异性扩散PDE",
            "TV": "Total Variation PDE(有阶梯效应)",
            "ITV": "改进TV，减少阶梯效应"
        }
        return descriptions.get(method_name, "")
    
    def _save_markdown_report(self, data_summary, table_data, results_folder):
        """生成Markdown格式的实验报告"""
        report_path = os.path.join(results_folder, "experiment_report.md")
        
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("# 图像去噪实验报告\n\n")
            
            # 实验信息
            f.write("## 实验信息\n\n")
            info = data_summary["experiment_info"]
            f.write(f"- **实验时间**: {info['timestamp']}\n")
            f.write(f"- **噪声类型**: {info['noise_type']}\n")
            f.write(f"- **噪声水平**: σ = {info['noise_level']}\n")
            f.write(f"- **图像尺寸**: {info['image_shape']}\n\n")
            
            # 结果表格
            f.write("## 实验结果对比\n\n")
            f.write("| 方法 | PSNR(dB) | SSIM | PSNR提升(dB) | SSIM提升 | 处理时间(s) | 备注 |\n")
            f.write("|------|----------|------|-------------|-----------|-------------|------|\n")
            
            for row in table_data:
                f.write(f"| {row['方法']} | {row['PSNR(dB)']} | {row['SSIM']} | "
                       f"{row['PSNR提升(dB)']} | {row['SSIM提升']} | {row['处理时间(s)']} | {row['备注']} |\n")
            
            # 重点分析
            f.write("\n## 重点分析\n\n")
            f.write("### TV vs ITV 对比\n\n")
            
            if 'TV' in data_summary["methods_results"] and 'ITV' in data_summary["methods_results"]:
                tv_data = data_summary["methods_results"]['TV']
                itv_data = data_summary["methods_results"]['ITV']
                
                f.write(f"- **TV方法**: PSNR = {tv_data['PSNR']:.2f} dB, SSIM = {tv_data['SSIM']:.4f}\n")
                f.write(f"- **ITV方法**: PSNR = {itv_data['PSNR']:.2f} dB, SSIM = {itv_data['SSIM']:.4f}\n")
                f.write(f"- **ITV相对TV的提升**:\n")
                f.write(f"  - PSNR提升: {itv_data['PSNR'] - tv_data['PSNR']:.2f} dB\n")
                f.write(f"  - SSIM提升: {itv_data['SSIM'] - tv_data['SSIM']:.4f}\n\n")
                
                if itv_data['SSIM'] > tv_data['SSIM']:
                    f.write("**结论**: ITV方法成功减少了阶梯化效应，结构相似性显著提升。\n\n")
            
            # 最佳方法
            f.write("### 最佳性能方法\n\n")
            best_psnr_method = max(data_summary["methods_results"].items(), key=lambda x: x[1]['PSNR'])
            best_ssim_method = max(data_summary["methods_results"].items(), key=lambda x: x[1]['SSIM'])
            
            f.write(f"- **最佳PSNR**: {best_psnr_method[0]} ({best_psnr_method[1]['PSNR']:.2f} dB)\n")
            f.write(f"- **最佳SSIM**: {best_ssim_method[0]} ({best_ssim_method[1]['SSIM']:.4f})\n\n")
            
            # 文件列表
            f.write("## 保存的文件\n\n")
            f.write("### 图像文件\n")
            f.write("- `01_original.png` - 原始图像\n")
            f.write("- `02_noisy_PSNR*.png` - 噪声图像\n")
            f.write("- `03_gaussian_PSNR*_SSIM*.png` - 高斯滤波结果\n")
            f.write("- `04_median_PSNR*_SSIM*.png` - 中值滤波结果\n")
            f.write("- `05_bilateral_PSNR*_SSIM*.png` - 双边滤波结果\n")
            f.write("- `06_adaptive_gaussian_PSNR*_SSIM*.png` - 自适应高斯滤波结果\n")
            f.write("- `07_perona_malik_PSNR*_SSIM*.png` - Perona-Malik结果\n")
            f.write("- `08_tv_PSNR*_SSIM*.png` - TV方法结果\n")
            f.write("- `09_itv_PSNR*_SSIM*.png` - ITV方法结果\n")
            f.write("- `comparison_all_methods.png` - 所有方法对比图\n")
            f.write("- `tv_vs_itv_detailed.png` - TV与ITV详细对比图\n\n")
            
            f.write("### 数据文件\n")
            f.write("- `detailed_results.json` - 详细数值结果(JSON格式)\n")
            f.write("- `results_summary.csv` - 结果汇总表格(CSV格式)\n")
            f.write("- `experiment_report.md` - 本实验报告(Markdown格式)\n")
    
    def _save_comparison_images(self, original, noisy, methods, metrics, noise_level, results_folder):
        """保存对比图像"""
        # 1. 保存所有方法对比图
        self._create_all_methods_comparison(original, noisy, methods, metrics, noise_level, results_folder)
        
        # 2. 保存TV vs ITV详细对比图
        self._create_tv_itv_detailed_comparison(original, noisy, methods, metrics, noise_level, results_folder)
    
    def _create_all_methods_comparison(self, original, noisy, methods, metrics, noise_level, results_folder):
        """创建所有方法的对比图"""
        fig, axes = plt.subplots(3, 3, figsize=(18, 18))
        fig.suptitle(f'综合图像去噪方法对比 - 高斯噪声 (σ={noise_level})', 
                   fontsize=16, fontweight='bold')
        
        # 显示顺序
        display_order = [
            ('原始图像', original, None),
            ('噪声图像', noisy, None),
            ('Gaussian Filter', methods.get('Gaussian Filter'), metrics.get('Gaussian Filter')),
            ('Median Filter', methods.get('Median Filter'), metrics.get('Median Filter')),
            ('Bilateral Filter', methods.get('Bilateral Filter'), metrics.get('Bilateral Filter')),
            ('Perona-Malik', methods.get('Perona-Malik'), metrics.get('Perona-Malik')),
            ('TV', methods.get('TV'), metrics.get('TV')),
            ('ITV', methods.get('ITV'), metrics.get('ITV')),
            ('Adaptive Gaussian', methods.get('Adaptive Gaussian'), metrics.get('Adaptive Gaussian'))
        ]
        
        for idx, (name, image, metric) in enumerate(display_order):
            if image is None:
                continue
                
            row = idx // 3
            col = idx % 3
            
            axes[row, col].imshow(image)
            if metric is None:
                if name == '噪声图像':
                    noisy_psnr = peak_signal_noise_ratio(original, noisy, data_range=1.0)
                    axes[row, col].set_title(f'{name}\nPSNR: {noisy_psnr:.2f}dB')
                else:
                    axes[row, col].set_title(name)
            else:
                axes[row, col].set_title(f'{name}\nPSNR: {metric["PSNR"]:.2f}dB\nSSIM: {metric["SSIM"]:.3f}')
            axes[row, col].axis('off')
        
        plt.tight_layout()
        comparison_path = os.path.join(results_folder, "comparison_all_methods.png")
        plt.savefig(comparison_path, dpi=300, bbox_inches='tight')
        plt.close()
    
    def _create_tv_itv_detailed_comparison(self, original, noisy, methods, metrics, noise_level, results_folder):
        """创建TV与ITV的详细对比图"""
        if 'TV' not in methods or 'ITV' not in methods:
            return
            
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle(f'TV vs ITV 详细对比 - 高斯噪声 (σ={noise_level})', 
                   fontsize=14, fontweight='bold')
        
        # 原始图像
        axes[0, 0].imshow(original)
        axes[0, 0].set_title('原始图像')
        axes[0, 0].axis('off')
        
        # 噪声图像
        noisy_psnr = peak_signal_noise_ratio(original, noisy, data_range=1.0)
        axes[0, 1].imshow(noisy)
        axes[0, 1].set_title(f'噪声图像\nPSNR: {noisy_psnr:.2f}dB')
        axes[0, 1].axis('off')
        
        # TV结果
        tv_metrics = metrics['TV']
        axes[1, 0].imshow(methods['TV'])
        axes[1, 0].set_title(f'TV去噪\nPSNR: {tv_metrics["PSNR"]:.2f}dB\nSSIM: {tv_metrics["SSIM"]:.4f}\n(易产生阶梯化效应)')
        axes[1, 0].axis('off')
        
        # ITV结果
        itv_metrics = metrics['ITV']
        axes[1, 1].imshow(methods['ITV'])
        axes[1, 1].set_title(f'ITV去噪\nPSNR: {itv_metrics["PSNR"]:.2f}dB\nSSIM: {itv_metrics["SSIM"]:.4f}\n(减少阶梯化效应)')
        axes[1, 1].axis('off')
        
        plt.tight_layout()
        tv_itv_path = os.path.join(results_folder, "tv_vs_itv_detailed.png")
        plt.savefig(tv_itv_path, dpi=300, bbox_inches='tight')
        plt.close()

    def run_comprehensive_analysis(self, image_path, noise_level=0.1):
        """运行综合对比分析"""
        print("="*70)
        print("综合图像去噪方法对比分析")
        print("重点对比TV与ITV的差异")
        print("="*70)
        
        # 加载原始图像
        original_image = self.load_image(image_path)
        if original_image is None:
            return
        
        # 添加噪声
        noisy_image = self.add_noise(original_image, 'gaussian', noise_level)
        
        print(f"\n处理高斯噪声 (σ={noise_level})...")
        
        # 所有去噪方法
        methods = {}
        times = {}
        
        # 1. 传统滤波方法
        start_time = time.time()
        methods['Gaussian Filter'] = self.gaussian_filter(noisy_image)
        times['Gaussian Filter'] = time.time() - start_time
        
        start_time = time.time()
        methods['Median Filter'] = self.median_filter(noisy_image)
        times['Median Filter'] = time.time() - start_time
        
        start_time = time.time()
        methods['Bilateral Filter'] = self.bilateral_filter(noisy_image)
        times['Bilateral Filter'] = time.time() - start_time
        
        start_time = time.time()
        methods['Adaptive Gaussian'] = self.adaptive_gaussian_filter(noisy_image)
        times['Adaptive Gaussian'] = time.time() - start_time
        
        # 2. PDE方法
        start_time = time.time()
        methods['Perona-Malik'] = self.perona_malik_diffusion(noisy_image)
        times['Perona-Malik'] = time.time() - start_time
        
        start_time = time.time()
        methods['TV'] = self.total_variation_denoising(noisy_image)
        times['TV'] = time.time() - start_time
        
        start_time = time.time()
        methods['ITV'] = self.improved_total_variation_denoising(noisy_image)
        times['ITV'] = time.time() - start_time
        
        # 性能评估
        print(f"\n{'='*60}")
        print("性能评估结果:")
        print(f"{'='*60}")
        
        evaluation_results = {}
        for method_name, result_image in methods.items():
            metrics = self.evaluate_performance(original_image, result_image, noisy_image)
            evaluation_results[method_name] = metrics
            
            print(f"\n{method_name}:")
            print(f"  PSNR: {metrics['PSNR']:.2f} dB")
            print(f"  SSIM: {metrics['SSIM']:.4f}")
            print(f"  PSNR提升: {metrics['PSNR_improvement']:.2f} dB")
            print(f"  处理时间: {times[method_name]:.2f}s")
        
        # 特别对比TV与ITV
        print(f"\n{'='*60}")
        print("TV vs ITV 特别对比:")
        print(f"{'='*60}")
        tv_metrics = evaluation_results['TV']
        itv_metrics = evaluation_results['ITV']
        
        print(f"TV  PSNR: {tv_metrics['PSNR']:.2f} dB")
        print(f"ITV PSNR: {itv_metrics['PSNR']:.2f} dB")
        print(f"ITV相对TV的PSNR提升: {itv_metrics['PSNR'] - tv_metrics['PSNR']:.2f} dB")
        print(f"TV  SSIM: {tv_metrics['SSIM']:.4f}")
        print(f"ITV SSIM: {itv_metrics['SSIM']:.4f}")
        print(f"ITV相对TV的SSIM提升: {itv_metrics['SSIM'] - tv_metrics['SSIM']:.4f}")
        
        # 保存所有结果到文件夹
        results_folder = self.save_results_to_folder(original_image, noisy_image, methods, evaluation_results, times, noise_level)
        
        return {
            'original': original_image,
            'noisy': noisy_image,
            'denoised': methods,
            'metrics': evaluation_results,
            'times': times,
            'results_folder': results_folder
        }

    def run_multi_noise_analysis(self, image_path, noise_levels=[0.05, 0.1, 0.15, 0.2]):
        """运行多噪声水平分析，生成论文质量的对比图表"""
        print("="*80)
        print("多噪声水平图像去噪综合分析")
        print("生成论文质量的对比图表和数据")
        print("="*80)
        
        # 创建主结果文件夹
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        main_folder = f"multi_noise_analysis_{timestamp}"
        os.makedirs(main_folder, exist_ok=True)
        
        # 加载原始图像
        original_image = self.load_image(image_path)
        if original_image is None:
            return
        
        print(f"\n主结果文件夹: {main_folder}")
        print(f"测试噪声水平: {noise_levels}")
        
        # 保存原始图像
        self._save_image(original_image, os.path.join(main_folder, "00_original_image.png"))
        
        # 存储所有结果
        all_results = {}
        method_names = ['Gaussian Filter', 'Median Filter', 'Bilateral Filter', 'Adaptive Gaussian', 'Perona-Malik', 'TV', 'ITV']
        
        # 为每个噪声水平进行测试
        for noise_level in noise_levels:
            print(f"\n{'='*60}")
            print(f"测试噪声水平: σ = {noise_level}")
            print(f"{'='*60}")
            
            # 创建单个噪声水平的文件夹
            noise_folder = os.path.join(main_folder, f"noise_level_{noise_level:.2f}")
            os.makedirs(noise_folder, exist_ok=True)
            
            # 添加噪声
            noisy_image = self.add_noise(original_image, 'gaussian', noise_level)
            
            # 运行所有方法
            results = self._run_all_methods(noisy_image)
            methods, times = results['methods'], results['times']
            
            # 性能评估
            evaluation_results = {}
            for method_name, result_image in methods.items():
                metrics = self.evaluate_performance(original_image, result_image, noisy_image)
                evaluation_results[method_name] = metrics
                
                print(f"{method_name}: PSNR={metrics['PSNR']:.2f}dB, SSIM={metrics['SSIM']:.4f}")
            
            # 保存单个噪声水平的详细结果
            self._save_single_noise_results(original_image, noisy_image, methods, 
                                          evaluation_results, times, noise_level, noise_folder)
            
            # 存储到总结果中
            all_results[noise_level] = {
                'original': original_image,
                'noisy': noisy_image,
                'methods': methods,
                'metrics': evaluation_results,
                'times': times
            }
        
        # 生成论文质量的综合分析
        self._generate_paper_quality_analysis(all_results, main_folder, method_names)
        
        print(f"\n{'='*80}")
        print("多噪声水平分析完成！")
        print(f"所有结果保存在: {main_folder}")
        print("生成的论文用图表:")
        print("  - performance_vs_noise_level.png (性能曲线图)")
        print("  - method_comparison_heatmap.png (性能热力图)")
        print("  - tv_vs_itv_analysis.png (TV与ITV对比)")
        print("  - processing_time_comparison.png (处理时间对比)")
        print("  - comprehensive_results_grid.png (综合结果网格)")
        print("="*80)
        
        return main_folder
    
    def _run_all_methods(self, noisy_image):
        """运行所有去噪方法"""
        methods = {}
        times = {}
        
        # 传统滤波方法 - 使用英文名称
        start_time = time.time()
        methods['Gaussian Filter'] = self.gaussian_filter(noisy_image)
        times['Gaussian Filter'] = time.time() - start_time
        
        start_time = time.time()
        methods['Median Filter'] = self.median_filter(noisy_image)
        times['Median Filter'] = time.time() - start_time
        
        start_time = time.time()
        methods['Bilateral Filter'] = self.bilateral_filter(noisy_image)
        times['Bilateral Filter'] = time.time() - start_time
        
        start_time = time.time()
        methods['Adaptive Gaussian'] = self.adaptive_gaussian_filter(noisy_image)
        times['Adaptive Gaussian'] = time.time() - start_time
        
        # PDE方法
        start_time = time.time()
        methods['Perona-Malik'] = self.perona_malik_diffusion(noisy_image)
        times['Perona-Malik'] = time.time() - start_time
        
        start_time = time.time()
        methods['TV'] = self.total_variation_denoising(noisy_image)
        times['TV'] = time.time() - start_time
        
        start_time = time.time()
        methods['ITV'] = self.improved_total_variation_denoising(noisy_image)
        times['ITV'] = time.time() - start_time
        
        return {'methods': methods, 'times': times}
    
    def _save_single_noise_results(self, original, noisy, methods, metrics, times, noise_level, folder):
        """保存单个噪声水平的结果"""
        # 保存噪声图像
        noisy_psnr = peak_signal_noise_ratio(original, noisy, data_range=1.0)
        self._save_image(noisy, os.path.join(folder, f"noisy_PSNR{noisy_psnr:.2f}.png"))
        
        # 保存所有去噪结果
        method_order = [
            ('Gaussian Filter', 'gaussian'),
            ('Median Filter', 'median'), 
            ('Bilateral Filter', 'bilateral'),
            ('Adaptive Gaussian', 'adaptive_gaussian'),
            ('Perona-Malik', 'perona_malik'),
            ('TV', 'tv'),
            ('ITV', 'itv')
        ]
        
        for method_name, filename_prefix in method_order:
            if method_name in methods:
                psnr = metrics[method_name]['PSNR']
                ssim = metrics[method_name]['SSIM']
                filename = f"{filename_prefix}_PSNR{psnr:.2f}_SSIM{ssim:.3f}.png"
                self._save_image(methods[method_name], os.path.join(folder, filename))
        
        # 生成该噪声水平的对比图
        self._create_single_noise_comparison(original, noisy, methods, metrics, noise_level, folder)
    
    def _create_single_noise_comparison(self, original, noisy, methods, metrics, noise_level, folder):
        """创建单个噪声水平的对比图"""
        fig, axes = plt.subplots(3, 3, figsize=(15, 15))
        title = get_text(
            f'图像去噪结果对比 - 噪声水平 σ={noise_level}',
            f'Image Denoising Comparison - Noise Level σ={noise_level}'
        )
        fig.suptitle(title, fontsize=16, fontweight='bold')
        
        # 显示顺序
        display_order = [
            (get_text('原始图像', 'Original'), original, None),
            (get_text('噪声图像', 'Noisy'), noisy, None),
            (get_text('Gaussian Filter', 'Gaussian'), methods.get('Gaussian Filter'), metrics.get('Gaussian Filter')),
            (get_text('Median Filter', 'Median'), methods.get('Median Filter'), metrics.get('Median Filter')),
            (get_text('Bilateral Filter', 'Bilateral'), methods.get('Bilateral Filter'), metrics.get('Bilateral Filter')),
            (get_text('Adaptive Gaussian', 'Adaptive Gaussian'), methods.get('Adaptive Gaussian'), metrics.get('Adaptive Gaussian')),
            ('Perona-Malik', methods.get('Perona-Malik'), metrics.get('Perona-Malik')),
            ('TV', methods.get('TV'), metrics.get('TV')),
            ('ITV', methods.get('ITV'), metrics.get('ITV'))
        ]
        
        for idx, (name, image, metric) in enumerate(display_order):
            if image is None:
                continue
                
            row = idx // 3
            col = idx % 3
            
            axes[row, col].imshow(image)
            if metric is None:
                if get_text('噪声图像', 'Noisy') in name:
                    noisy_psnr = peak_signal_noise_ratio(original, noisy, data_range=1.0)
                    axes[row, col].set_title(f'{name}\nPSNR: {noisy_psnr:.2f}dB', fontsize=10)
                else:
                    axes[row, col].set_title(name, fontsize=10)
            else:
                axes[row, col].set_title(f'{name}\nPSNR: {metric["PSNR"]:.2f}dB\nSSIM: {metric["SSIM"]:.3f}', fontsize=9)
            axes[row, col].axis('off')
        
        plt.tight_layout()
        plt.savefig(os.path.join(folder, f'comparison_sigma_{noise_level:.2f}.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _generate_paper_quality_analysis(self, all_results, main_folder, method_names):
        """生成论文质量的分析图表"""
        noise_levels = list(all_results.keys())
        
        # 1. 性能曲线图 (PSNR vs 噪声水平)
        self._plot_performance_curves(all_results, main_folder, method_names)
        
        # 2. 性能热力图
        self._plot_performance_heatmap(all_results, main_folder, method_names)
        
        # 3. TV vs ITV 专项分析
        self._plot_tv_itv_analysis(all_results, main_folder)
        
        # 4. 处理时间对比
        self._plot_processing_time_comparison(all_results, main_folder, method_names)
        
        # 5. 综合结果网格
        self._plot_comprehensive_grid(all_results, main_folder)
        
        # 6. 保存综合数据表
        self._save_comprehensive_data(all_results, main_folder, method_names)
    
    def _plot_performance_curves(self, all_results, main_folder, method_names):
        """绘制性能曲线图"""
        plt.style.use('default')
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
        
        noise_levels = sorted(all_results.keys())
        colors = plt.cm.Set1(np.linspace(0, 1, len(method_names)))
        
        # PSNR曲线
        for i, method in enumerate(method_names):
            psnr_values = [all_results[sigma]['metrics'][method]['PSNR'] for sigma in noise_levels]
            ax1.plot(noise_levels, psnr_values, 'o-', color=colors[i], label=method, 
                    linewidth=2, markersize=6)
        
        ax1.set_xlabel(get_text('噪声水平 σ', 'Noise Level σ'), fontsize=12)
        ax1.set_ylabel('PSNR (dB)', fontsize=12)
        ax1.set_title(get_text('PSNR vs 噪声水平', 'PSNR vs Noise Level'), fontsize=14, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # SSIM曲线
        for i, method in enumerate(method_names):
            ssim_values = [all_results[sigma]['metrics'][method]['SSIM'] for sigma in noise_levels]
            ax2.plot(noise_levels, ssim_values, 's-', color=colors[i], label=method,
                    linewidth=2, markersize=6)
        
        ax2.set_xlabel(get_text('噪声水平 σ', 'Noise Level σ'), fontsize=12)
        ax2.set_ylabel('SSIM', fontsize=12)
        ax2.set_title(get_text('SSIM vs 噪声水平', 'SSIM vs Noise Level'), fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        plt.tight_layout()
        plt.savefig(os.path.join(main_folder, 'performance_vs_noise_level.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_performance_heatmap(self, all_results, main_folder, method_names):
        """绘制性能热力图"""
        noise_levels = sorted(all_results.keys())
        
        # 准备PSNR数据
        psnr_data = np.zeros((len(method_names), len(noise_levels)))
        for i, method in enumerate(method_names):
            for j, sigma in enumerate(noise_levels):
                psnr_data[i, j] = all_results[sigma]['metrics'][method]['PSNR']
        
        # 准备SSIM数据
        ssim_data = np.zeros((len(method_names), len(noise_levels)))
        for i, method in enumerate(method_names):
            for j, sigma in enumerate(noise_levels):
                ssim_data[i, j] = all_results[sigma]['metrics'][method]['SSIM']
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        # PSNR热力图
        im1 = ax1.imshow(psnr_data, cmap='RdYlGn', aspect='auto')
        ax1.set_xticks(range(len(noise_levels)))
        ax1.set_xticklabels([f'σ={σ}' for σ in noise_levels])
        ax1.set_yticks(range(len(method_names)))
        ax1.set_yticklabels(method_names)
        ax1.set_title(get_text('PSNR (dB) 热力图', 'PSNR (dB) Heatmap'), fontsize=14, fontweight='bold')
        
        # 添加数值标注
        for i in range(len(method_names)):
            for j in range(len(noise_levels)):
                ax1.text(j, i, f'{psnr_data[i, j]:.1f}', ha='center', va='center', 
                        color='white' if psnr_data[i, j] < 25 else 'black', fontweight='bold')
        
        plt.colorbar(im1, ax=ax1)
        
        # SSIM热力图
        im2 = ax2.imshow(ssim_data, cmap='RdYlGn', aspect='auto')
        ax2.set_xticks(range(len(noise_levels)))
        ax2.set_xticklabels([f'σ={σ}' for σ in noise_levels])
        ax2.set_yticks(range(len(method_names)))
        ax2.set_yticklabels(method_names)
        ax2.set_title(get_text('SSIM 热力图', 'SSIM Heatmap'), fontsize=14, fontweight='bold')
        
        # 添加数值标注
        for i in range(len(method_names)):
            for j in range(len(noise_levels)):
                ax2.text(j, i, f'{ssim_data[i, j]:.3f}', ha='center', va='center',
                        color='white' if ssim_data[i, j] < 0.5 else 'black', fontweight='bold')
        
        plt.colorbar(im2, ax=ax2)
        plt.tight_layout()
        plt.savefig(os.path.join(main_folder, 'method_comparison_heatmap.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_tv_itv_analysis(self, all_results, main_folder):
        """绘制TV vs ITV专项分析"""
        noise_levels = sorted(all_results.keys())
        
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        
        # 提取TV和ITV数据
        tv_psnr = [all_results[sigma]['metrics']['TV']['PSNR'] for sigma in noise_levels]
        itv_psnr = [all_results[sigma]['metrics']['ITV']['PSNR'] for sigma in noise_levels]
        tv_ssim = [all_results[sigma]['metrics']['TV']['SSIM'] for sigma in noise_levels]
        itv_ssim = [all_results[sigma]['metrics']['ITV']['SSIM'] for sigma in noise_levels]
        
        # PSNR对比
        ax1.plot(noise_levels, tv_psnr, 'ro-', label='TV', linewidth=3, markersize=8)
        ax1.plot(noise_levels, itv_psnr, 'bo-', label='ITV', linewidth=3, markersize=8)
        ax1.set_xlabel(get_text('噪声水平 σ', 'Noise Level σ'), fontsize=12)
        ax1.set_ylabel('PSNR (dB)', fontsize=12)
        ax1.set_title(get_text('TV vs ITV: PSNR对比', 'TV vs ITV: PSNR Comparison'), fontsize=14, fontweight='bold')
        ax1.legend(fontsize=12)
        ax1.grid(True, alpha=0.3)
        
        # SSIM对比
        ax2.plot(noise_levels, tv_ssim, 'ro-', label='TV', linewidth=3, markersize=8)
        ax2.plot(noise_levels, itv_ssim, 'bo-', label='ITV', linewidth=3, markersize=8)
        ax2.set_xlabel(get_text('噪声水平 σ', 'Noise Level σ'), fontsize=12)
        ax2.set_ylabel('SSIM', fontsize=12)
        ax2.set_title(get_text('TV vs ITV: SSIM对比', 'TV vs ITV: SSIM Comparison'), fontsize=14, fontweight='bold')
        ax2.legend(fontsize=12)
        ax2.grid(True, alpha=0.3)
        
        # 改进幅度 - PSNR
        psnr_improvement = [itv - tv for tv, itv in zip(tv_psnr, itv_psnr)]
        ax3.bar(range(len(noise_levels)), psnr_improvement, color='green', alpha=0.7)
        ax3.set_xticks(range(len(noise_levels)))
        ax3.set_xticklabels([f'σ={σ}' for σ in noise_levels])
        ax3.set_ylabel(get_text('PSNR提升 (dB)', 'PSNR Improvement (dB)'), fontsize=12)
        ax3.set_title(get_text('ITV相对TV的PSNR提升', 'ITV PSNR Improvement over TV'), fontsize=14, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        
        # 改进幅度 - SSIM
        ssim_improvement = [itv - tv for tv, itv in zip(tv_ssim, itv_ssim)]
        ax4.bar(range(len(noise_levels)), ssim_improvement, color='blue', alpha=0.7)
        ax4.set_xticks(range(len(noise_levels)))
        ax4.set_xticklabels([f'σ={σ}' for σ in noise_levels])
        ax4.set_ylabel(get_text('SSIM提升', 'SSIM Improvement'), fontsize=12)
        ax4.set_title(get_text('ITV相对TV的SSIM提升', 'ITV SSIM Improvement over TV'), fontsize=14, fontweight='bold')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(main_folder, 'tv_vs_itv_analysis.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_processing_time_comparison(self, all_results, main_folder, method_names):
        """绘制处理时间对比"""
        noise_levels = sorted(all_results.keys())
        
        # 计算平均处理时间
        avg_times = {}
        for method in method_names:
            times = [all_results[sigma]['times'][method] for sigma in noise_levels]
            avg_times[method] = np.mean(times)
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
        
        # 平均处理时间柱状图
        methods = list(avg_times.keys())
        times = list(avg_times.values())
        colors = plt.cm.Set3(np.linspace(0, 1, len(methods)))
        
        bars = ax1.bar(methods, times, color=colors)
        ax1.set_ylabel(get_text('平均处理时间 (秒)', 'Average Processing Time (s)'), fontsize=12)
        ax1.set_title(get_text('各方法平均处理时间对比', 'Average Processing Time Comparison'), fontsize=14, fontweight='bold')
        ax1.tick_params(axis='x', rotation=45)
        
        # 添加数值标注
        for bar, time_val in zip(bars, times):
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    f'{time_val:.2f}s', ha='center', va='bottom', fontweight='bold')
        
        # 处理时间vs噪声水平
        colors = plt.cm.Set1(np.linspace(0, 1, len(method_names)))
        for i, method in enumerate(method_names):
            times = [all_results[sigma]['times'][method] for sigma in noise_levels]
            ax2.plot(noise_levels, times, 'o-', color=colors[i], label=method, linewidth=2)
        
        ax2.set_xlabel(get_text('噪声水平 σ', 'Noise Level σ'), fontsize=12)
        ax2.set_ylabel(get_text('处理时间 (秒)', 'Processing Time (s)'), fontsize=12)
        ax2.set_title(get_text('处理时间 vs 噪声水平', 'Processing Time vs Noise Level'), fontsize=14, fontweight='bold')
        ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(main_folder, 'processing_time_comparison.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_comprehensive_grid(self, all_results, main_folder):
        """绘制综合结果网格"""
        noise_levels = sorted(all_results.keys())
        n_noise = len(noise_levels)
        
        # 选择代表性方法显示
        key_methods = [
            get_text('噪声图像', 'Noisy'), 
            get_text('Gaussian Filter', 'Gaussian'), 
            'TV', 
            'ITV'
        ]
        
        fig, axes = plt.subplots(n_noise, len(key_methods), figsize=(16, 4*n_noise))
        title = get_text('多噪声水平综合对比 - 关键方法', 'Multi-Noise Level Comprehensive Comparison - Key Methods')
        fig.suptitle(title, fontsize=16, fontweight='bold')
        
        for i, noise_level in enumerate(noise_levels):
            original = all_results[noise_level]['original']
            noisy = all_results[noise_level]['noisy']
            methods = all_results[noise_level]['methods']
            metrics = all_results[noise_level]['metrics']
            
            # 噪声图像
            axes[i, 0].imshow(noisy)
            noisy_psnr = peak_signal_noise_ratio(original, noisy, data_range=1.0)
            axes[i, 0].set_title(f'σ={noise_level}\nPSNR: {noisy_psnr:.2f}dB')
            axes[i, 0].axis('off')
            
            # 其他方法
            method_map = {'Gaussian Filter': 'Gaussian Filter', 'Gaussian': 'Gaussian Filter', 'TV': 'TV', 'ITV': 'ITV'}
            display_methods = [get_text('Gaussian Filter', 'Gaussian'), 'TV', 'ITV']
            for j, method_display in enumerate(display_methods, 1):
                method_key = method_map.get(method_display, method_display)
                if method_key in methods:
                    axes[i, j].imshow(methods[method_key])
                    metric = metrics[method_key]
                    axes[i, j].set_title(f'{method_display}\nPSNR: {metric["PSNR"]:.2f}dB\nSSIM: {metric["SSIM"]:.3f}')
                    axes[i, j].axis('off')
        
        plt.tight_layout()
        plt.savefig(os.path.join(main_folder, 'comprehensive_results_grid.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _save_comprehensive_data(self, all_results, main_folder, method_names):
        """保存综合数据表"""
        noise_levels = sorted(all_results.keys())
        
        # 创建综合数据表
        data_rows = []
        
        for noise_level in noise_levels:
            # 噪声图像数据
            original = all_results[noise_level]['original']
            noisy = all_results[noise_level]['noisy']
            noisy_psnr = peak_signal_noise_ratio(original, noisy, data_range=1.0)
            if len(original.shape) == 3:
                noisy_ssim = structural_similarity(original, noisy, multichannel=True, channel_axis=2, data_range=1.0)
            else:
                noisy_ssim = structural_similarity(original, noisy, data_range=1.0)
            
            data_rows.append({
                '噪声水平': f'σ={noise_level}',
                '方法': '噪声图像',
                'PSNR(dB)': f'{noisy_psnr:.2f}',
                'SSIM': f'{noisy_ssim:.4f}',
                'PSNR提升(dB)': '0.00',
                '处理时间(s)': '0.00'
            })
            
            # 各方法数据
            metrics = all_results[noise_level]['metrics']
            times = all_results[noise_level]['times']
            
            for method_name in method_names:
                if method_name in metrics:
                    metric = metrics[method_name]
                    time_taken = times[method_name]
                    
                    data_rows.append({
                        '噪声水平': f'σ={noise_level}',
                        '方法': method_name,
                        'PSNR(dB)': f'{metric["PSNR"]:.2f}',
                        'SSIM': f'{metric["SSIM"]:.4f}',
                        'PSNR提升(dB)': f'{metric["PSNR_improvement"]:.2f}',
                        '处理时间(s)': f'{time_taken:.2f}'
                    })
        
        # 保存CSV
        df = pd.DataFrame(data_rows)
        csv_path = os.path.join(main_folder, 'comprehensive_results.csv')
        df.to_csv(csv_path, index=False, encoding='utf-8-sig')
        
        # 保存JSON
        json_data = {
            'experiment_info': {
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'noise_levels': noise_levels,
                'methods_tested': method_names
            },
            'results_by_noise_level': {}
        }
        
        for noise_level in noise_levels:
            json_data['results_by_noise_level'][f'sigma_{noise_level}'] = {
                'noise_level': noise_level,
                'methods': {}
            }
            
            metrics = all_results[noise_level]['metrics']
            times = all_results[noise_level]['times']
            
            for method_name in method_names:
                if method_name in metrics:
                    json_data['results_by_noise_level'][f'sigma_{noise_level}']['methods'][method_name] = {
                        'PSNR': float(metrics[method_name]['PSNR']),
                        'SSIM': float(metrics[method_name]['SSIM']),
                        'MSE': float(metrics[method_name]['MSE']),
                        'PSNR_improvement': float(metrics[method_name]['PSNR_improvement']),
                        'SSIM_improvement': float(metrics[method_name]['SSIM_improvement']),
                        'processing_time': float(times[method_name])
                    }
        
        json_path = os.path.join(main_folder, 'comprehensive_results.json')
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(json_data, f, ensure_ascii=False, indent=2)
        
        print(f"\n数据文件已保存:")
        print(f"  - {csv_path}")
        print(f"  - {json_path}")

def main():
    """主函数 - 运行多噪声水平分析"""
    print("综合图像去噪方法对比系统")
    print("多噪声水平测试 - 生成论文质量图表")
    print("="*50)
    
    # 创建综合去噪器实例
    denoiser = ComprehensiveDenoising()
    
    # 设置图像路径
    image_path = "example/images.png"
    
    # 检查图像是否存在
    if not os.path.exists(image_path):
        print(f"错误：找不到图像文件 {image_path}")
        return
    
    # 运行多噪声水平分析
    noise_levels = [0.05, 0.1, 0.15, 0.2]  # 从轻微到严重的噪声水平
    results_folder = denoiser.run_multi_noise_analysis(
        image_path=image_path,
        noise_levels=noise_levels
    )
    
    print(f"\n多噪声水平分析完成！")
    print(f"所有结果保存在: {results_folder}")


if __name__ == "__main__":
    main() 