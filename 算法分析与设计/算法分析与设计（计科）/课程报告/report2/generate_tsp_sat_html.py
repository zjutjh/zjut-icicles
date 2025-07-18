import os
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.chrome.service import Service
from webdriver_manager.chrome import ChromeDriverManager
import time
import base64

# HTML模板
html_template = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>TSP与SAT问题的关系</title>
    <style>
        body {
            font-family: SimSun, "Microsoft YaHei", Arial, sans-serif;
            background-color: #f8f8f8;
            margin: 0;
            padding: 20px;
        }
        .container {
            width: 1000px;
            height: 800px;
            position: relative;
            margin: 0 auto;
        }
        .title {
            text-align: center;
            font-size: 24px;
            font-weight: bold;
            margin-bottom: 30px;
        }
        .tsp-example {
            position: absolute;
            top: 100px;
            left: 100px;
            width: 300px;
            height: 250px;
            background-color: #D6EAF8;
            border: 1px solid #85C1E9;
            border-radius: 10px;
            padding: 10px;
            text-align: center;
        }
        .sat-example {
            position: absolute;
            top: 100px;
            right: 100px;
            width: 300px;
            height: 250px;
            background-color: #E6F5D0;
            border: 1px solid #A9DFBF;
            border-radius: 10px;
            padding: 10px;
            text-align: center;
        }
        .reduction {
            position: absolute;
            top: 225px;
            left: 420px;
            width: 160px;
            text-align: center;
        }
        .arrow {
            width: 100%;
            height: 20px;
            position: relative;
        }
        .arrow:after {
            content: '';
            position: absolute;
            top: 50%;
            right: 0;
            width: 0;
            height: 0;
            border: 10px solid transparent;
            border-left-color: #E74C3C;
            transform: translateY(-50%);
        }
        .arrow:before {
            content: '';
            position: absolute;
            top: 50%;
            left: 0;
            right: 10px;
            height: 2px;
            background-color: #E74C3C;
            transform: translateY(-50%);
        }
        .reduction-label {
            color: #E74C3C;
            font-weight: bold;
            margin-top: 5px;
        }
        .variables {
            position: absolute;
            top: 280px;
            left: 350px;
            width: 300px;
            background-color: white;
            border: 1px solid #D5D8DC;
            border-radius: 5px;
            padding: 10px;
            text-align: left;
        }
        .significance {
            position: absolute;
            bottom: 50px;
            left: 250px;
            width: 500px;
            background-color: #FFE6CC;
            border: 1px solid #F5B041;
            border-radius: 10px;
            padding: 15px;
            text-align: center;
        }
        .significance h3 {
            margin-top: 0;
        }
        .tsp-graph {
            width: 100%;
            height: 150px;
            position: relative;
            margin-top: 10px;
        }
        .tsp-node {
            width: 30px;
            height: 30px;
            border-radius: 50%;
            background-color: skyblue;
            border: 2px solid black;
            position: absolute;
            display: flex;
            align-items: center;
            justify-content: center;
            font-weight: bold;
        }
        .tsp-edge {
            position: absolute;
            height: 2px;
            background-color: black;
            transform-origin: 0 0;
        }
        .tsp-weight {
            position: absolute;
            background-color: white;
            border-radius: 50%;
            padding: 2px 5px;
            font-size: 12px;
        }
        .section-title {
            font-weight: bold;
            margin-bottom: 10px;
        }
        .description {
            background-color: white;
            border-radius: 5px;
            padding: 5px;
            margin-top: 10px;
            font-size: 14px;
        }
        ul {
            text-align: left;
            margin: 10px 0;
            padding-left: 20px;
        }
        .formula {
            background-color: white;
            border-radius: 5px;
            padding: 10px;
            margin: 10px 0;
            font-family: "Courier New", monospace;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="title">TSP与SAT问题的关系</div>
        
        <!-- TSP示例 -->
        <div class="tsp-example">
            <div class="section-title">TSP问题示例</div>
            <div class="tsp-graph">
                <!-- 节点 -->
                <div class="tsp-node" style="top: 50px; left: 50px;">A</div>
                <div class="tsp-node" style="top: 10px; left: 100px;">B</div>
                <div class="tsp-node" style="top: 30px; left: 150px;">C</div>
                <div class="tsp-node" style="top: 10px; left: 200px;">D</div>
                <div class="tsp-node" style="top: 50px; left: 250px;">E</div>
                
                <!-- 这里省略了边的绘制，实际应用中需要添加 -->
            </div>
            <div class="description">
                目标：找出访问所有城市并返回起点的最短路径
            </div>
        </div>
        
        <!-- SAT示例 -->
        <div class="sat-example">
            <div class="section-title">SAT问题示例</div>
            <div class="formula">
                (x1 OR x2 OR NOT x3) AND<br>
                (NOT x1 OR NOT x2 OR x4) AND<br>
                (x2 OR NOT x3 OR NOT x4) AND<br>
                (x1 OR x3 OR x4)
            </div>
            <div class="description">
                目标：找出使公式为真的变量赋值
            </div>
        </div>
        
        <!-- 归约过程 -->
        <div class="reduction">
            <div class="arrow"></div>
            <div class="reduction-label">多项式时间归约</div>
        </div>
        
        <!-- 变量设计 -->
        <div class="variables">
            <strong>变量设计:</strong>
            <ul>
                <li>x[i,j,p]: 城市i在位置p访问城市j</li>
                <li>位置唯一性约束</li>
                <li>访问唯一性约束</li>
                <li>路径连续性约束</li>
                <li>路径长度约束</li>
            </ul>
        </div>
        
        <!-- 理论意义 -->
        <div class="significance">
            <h3>理论意义</h3>
            <ul>
                <li>证明了TSP和SAT都是NP完全问题</li>
                <li>归约建立了问题之间的桥梁</li>
                <li>任何一个问题的多项式解法都意味着P=NP</li>
                <li>为算法设计提供了不同视角</li>
                <li>启发了混合求解策略的开发</li>
            </ul>
        </div>
    </div>
</body>
</html>
"""

def generate_html_image():
    # 保存HTML文件
    with open("tsp_sat_relation.html", "w", encoding="utf-8") as f:
        f.write(html_template)
    
    # 设置Chrome选项
    chrome_options = Options()
    chrome_options.add_argument("--headless")  # 无头模式
    chrome_options.add_argument("--disable-gpu")
    chrome_options.add_argument("--window-size=1200,1000")  # 设置窗口大小
    
    try:
        # 初始化WebDriver
        driver = webdriver.Chrome(service=Service(ChromeDriverManager().install()), options=chrome_options)
        
        # 打开HTML文件
        driver.get("file://" + os.path.abspath("tsp_sat_relation.html"))
        
        # 等待页面加载
        time.sleep(2)
        
        # 截图
        driver.save_screenshot("img/tsp_sat_relation.png")
        
        # 关闭WebDriver
        driver.quit()
        
        print("HTML图像已成功生成并保存为'img/tsp_sat_relation.png'")
        
    except Exception as e:
        print(f"生成图像时出错: {e}")
        
        # 如果无法使用Selenium，尝试提供HTML文件
        print("无法使用WebDriver生成图像，已保存HTML文件，您可以手动打开并截图。")

if __name__ == "__main__":
    # 确保img目录存在
    os.makedirs("img", exist_ok=True)
    
    # 生成图像
    generate_html_image() 