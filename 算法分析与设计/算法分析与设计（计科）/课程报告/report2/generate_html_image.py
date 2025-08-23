import os
import subprocess
import sys
import platform

def generate_html_image():
    """
    使用wkhtmltoimage将HTML转换为图像
    如果wkhtmltoimage不可用，提供说明
    """
    # 确保img目录存在
    os.makedirs("img", exist_ok=True)
    
    # 检查操作系统类型
    system = platform.system()
    
    # wkhtmltoimage命令
    if system == "Windows":
        wkhtmltoimage_cmd = "wkhtmltoimage"
    else:
        wkhtmltoimage_cmd = "wkhtmltoimage"
    
    try:
        # 尝试运行wkhtmltoimage
        subprocess.run([
            wkhtmltoimage_cmd,
            "--quality", "100",
            "--width", "1000",
            "tsp_sat_relation.html",
            "img/tsp_sat_relation.png"
        ], check=True)
        
        print("HTML图像已成功生成并保存为'img/tsp_sat_relation.png'")
        
    except FileNotFoundError:
        print("错误: 未找到wkhtmltoimage工具。")
        print("\n要安装wkhtmltoimage:")
        if system == "Windows":
            print("1. 访问 https://wkhtmltopdf.org/downloads.html")
            print("2. 下载并安装适用于Windows的版本")
            print("3. 确保wkhtmltoimage在系统PATH中")
        elif system == "Darwin":  # macOS
            print("1. 使用Homebrew安装: brew install wkhtmltopdf")
        elif system == "Linux":
            print("1. 使用包管理器安装，例如: sudo apt-get install wkhtmltopdf")
        
        print("\n或者，您可以手动打开HTML文件并使用浏览器截图:")
        print(f"HTML文件位置: {os.path.abspath('tsp_sat_relation.html')}")
        
    except Exception as e:
        print(f"生成图像时出错: {e}")
        print(f"您可以手动打开HTML文件并截图: {os.path.abspath('tsp_sat_relation.html')}")

if __name__ == "__main__":
    generate_html_image() 