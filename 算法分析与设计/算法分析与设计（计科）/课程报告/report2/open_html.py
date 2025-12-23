import os
import webbrowser
import platform
import subprocess

def open_html_file():
    """
    打开HTML文件并提供截图指导
    """
    html_path = os.path.abspath("tsp_sat_relation.html")
    
    # 打开HTML文件
    webbrowser.open('file://' + html_path)
    
    print("HTML文件已在浏览器中打开。")
    print("\n截图指导:")
    print("1. 等待页面完全加载")
    print("2. 使用浏览器的全屏模式(通常按F11键)")
    print("3. 使用截图工具(如Windows的截图工具、Mac的Command+Shift+4等)")
    print("4. 保存截图到img/tsp_sat_relation.png")
    
    # 确保img目录存在
    os.makedirs("img", exist_ok=True)
    
    # 打开文件夹以便保存截图
    if platform.system() == "Windows":
        subprocess.run(["explorer", os.path.abspath("img")])
    elif platform.system() == "Darwin":  # macOS
        subprocess.run(["open", os.path.abspath("img")])
    elif platform.system() == "Linux":
        subprocess.run(["xdg-open", os.path.abspath("img")])

if __name__ == "__main__":
    open_html_file() 