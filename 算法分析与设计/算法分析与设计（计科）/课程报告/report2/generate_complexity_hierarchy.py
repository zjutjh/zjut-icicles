import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'Microsoft YaHei', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False  # 正确显示负号

# 创建图形
fig, ax = plt.subplots(figsize=(12, 10))

# 设置背景颜色
ax.set_facecolor('#f8f8f8')

# 定义复杂性类的层次结构
classes = [
    {'name': 'EXPSPACE', 'color': '#FFD6D6', 'x': 0.5, 'y': 0.9, 'width': 0.9, 'height': 0.1, 
     'desc': '指数空间复杂性类: 空间复杂度为2^{poly(n)}的问题'},
    {'name': 'EXPTIME', 'color': '#FFE6CC', 'x': 0.5, 'y': 0.8, 'width': 0.85, 'height': 0.1,
     'desc': '指数时间复杂性类: 时间复杂度为2^{poly(n)}的问题'},
    {'name': 'PSPACE', 'color': '#D4E7FF', 'x': 0.5, 'y': 0.7, 'width': 0.8, 'height': 0.1,
     'desc': '多项式空间复杂性类: 空间复杂度为poly(n)的问题'},
    {'name': 'NP', 'color': '#D5E8D4', 'x': 0.5, 'y': 0.6, 'width': 0.75, 'height': 0.1,
     'desc': '非确定性多项式时间复杂性类: 可在非确定性图灵机上多项式时间内解决的问题'},
    {'name': 'P', 'color': '#DAE8FC', 'x': 0.5, 'y': 0.5, 'width': 0.6, 'height': 0.1,
     'desc': '多项式时间复杂性类: 可在确定性图灵机上多项式时间内解决的问题'},
    {'name': 'NL', 'color': '#E1D5E7', 'x': 0.5, 'y': 0.4, 'width': 0.45, 'height': 0.1,
     'desc': '非确定性对数空间复杂性类: 可在非确定性图灵机上对数空间内解决的问题'},
    {'name': 'L', 'color': '#FFF2CC', 'x': 0.5, 'y': 0.3, 'width': 0.3, 'height': 0.1,
     'desc': '对数空间复杂性类: 可在确定性图灵机上对数空间内解决的问题'},
    {'name': 'NC', 'color': '#F8CECC', 'x': 0.5, 'y': 0.2, 'width': 0.2, 'height': 0.1,
     'desc': '尼克类: 可在并行计算模型中多项式处理器上多项式对数时间内解决的问题'},
    {'name': 'AC0', 'color': '#B0E3E6', 'x': 0.5, 'y': 0.1, 'width': 0.1, 'height': 0.1,
     'desc': '常数深度交替电路类: 可由常数深度、无限扇入的AND/OR/NOT门电路解决的问题'}
]

# 绘制复杂性类层次结构
for cls in classes:
    x, y = cls['x'], cls['y']
    width, height = cls['width'], cls['height']
    
    # 绘制矩形
    rect = patches.Rectangle((x - width/2, y - height/2), width, height, 
                           facecolor=cls['color'], edgecolor='black', alpha=0.8, linewidth=1)
    ax.add_patch(rect)
    
    # 添加类名
    ax.text(x, y, cls['name'], ha='center', va='center', fontsize=14, fontweight='bold')
    
    # 添加描述
    if cls['name'] != 'AC⁰':  # 特殊处理AC⁰，因为它的矩形较小
        desc_y = y - height/2 - 0.01
        ax.text(x, desc_y, cls['desc'], ha='center', va='top', fontsize=8,
               bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.2'))

# 添加包含关系箭头
for i in range(len(classes) - 1):
    start_y = classes[i+1]['y'] + classes[i+1]['height']/2
    end_y = classes[i]['y'] - classes[i]['height']/2
    
    ax.annotate("", xy=(0.5, end_y), xytext=(0.5, start_y),
               arrowprops=dict(arrowstyle="->", lw=1.5, color='black', alpha=0.6))

# 添加重要问题标注
problems = [
    {'name': 'TQBF (真量化布尔公式)', 'class': 'PSPACE', 'x_offset': 0.25},
    {'name': 'SAT (布尔可满足性问题)', 'class': 'NP', 'x_offset': 0.25},
    {'name': 'TSP (旅行商问题)', 'class': 'NP', 'x_offset': -0.25},
    {'name': '矩阵乘法', 'class': 'P', 'x_offset': 0.2},
    {'name': '最短路径', 'class': 'P', 'x_offset': -0.2},
    {'name': '图可达性', 'class': 'NL', 'x_offset': 0.15},
    {'name': '2-SAT', 'class': 'NL', 'x_offset': -0.15},
    {'name': '确定性有限自动机', 'class': 'L', 'x_offset': 0.12},
    {'name': '排序', 'class': 'NC', 'x_offset': 0.08},
    {'name': '奇偶校验', 'class': 'AC⁰', 'x_offset': 0.04}
]

# 绘制问题标注
for problem in problems:
    # 找到对应的复杂性类
    cls = next((c for c in classes if c['name'] == problem['class']), None)
    if cls:
        x = cls['x'] + problem['x_offset']
        y = cls['y']
        
        # 绘制问题点
        ax.plot(x, y, 'o', markersize=6, color='red')
        
        # 添加问题名称
        if problem['x_offset'] > 0:
            ha = 'left'
            x_text = x + 0.02
        else:
            ha = 'right'
            x_text = x - 0.02
        
        ax.text(x_text, y, problem['name'], ha=ha, va='center', fontsize=8,
               bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.2'))

# 添加P vs NP问题标注
ax.text(0.5, 0.55, "P=NP?", ha='center', va='center', fontsize=16, color='red', fontweight='bold',
       bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.3'))

# 添加标题
ax.text(0.5, 0.95, "计算复杂性层次结构", fontsize=18, ha='center', fontweight='bold')

# 添加说明
explanation = (
    "注：此图展示了主要计算复杂性类之间的包含关系。\n"
    "较低的类被包含在较高的类中。P=NP问题是计算机科学中最重要的未解决问题之一，\n"
    "它询问P类和NP类是否相等。每个类中标注了一些代表性问题。"
)
ax.text(0.5, 0.02, explanation, ha='center', va='bottom', fontsize=10,
       bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.3'))

# 设置坐标轴范围
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)

# 隐藏坐标轴
ax.axis('off')

# 保存图片
plt.savefig('img/complexity_hierarchy.png', dpi=300, bbox_inches='tight')
plt.close()

print("计算复杂性层次结构图已保存为'img/complexity_hierarchy.png'") 