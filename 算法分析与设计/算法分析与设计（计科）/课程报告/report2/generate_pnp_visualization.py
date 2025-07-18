import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse, Circle, Rectangle
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False  # 正确显示负号

# 创建图形和坐标轴
plt.figure(figsize=(10, 8))
ax = plt.subplot(111, aspect='equal')

# 设置坐标轴范围
ax.set_xlim(-6, 6)
ax.set_ylim(-5, 5)

# 绘制NP类（大椭圆）
np_ellipse = Ellipse(xy=(0, 0), width=10, height=8, 
                    edgecolor='blue', facecolor='lightblue', 
                    alpha=0.3, label='NP类')
ax.add_patch(np_ellipse)

# 绘制P类（小椭圆）
p_ellipse = Ellipse(xy=(-2, 0), width=5, height=4, 
                   edgecolor='green', facecolor='lightgreen', 
                   alpha=0.5, label='P类')
ax.add_patch(p_ellipse)

# 绘制NP完全类（右侧区域）
np_complete = Ellipse(xy=(2.5, 0), width=4.5, height=3.5, 
                     edgecolor='red', facecolor='lightcoral', 
                     alpha=0.5, label='NP完全类')
ax.add_patch(np_complete)

# 绘制NP难问题（包括NP完全和更大范围）
np_hard_outer = Ellipse(xy=(3, 0), width=12, height=9, 
                       edgecolor='purple', facecolor='none', 
                       linestyle='--', label='NP难问题')
ax.add_patch(np_hard_outer)

# 添加问题示例
problems = [
    (-2.5, 1, "排序", "green"),
    (-1.5, -1, "最短路径", "green"),
    (-3, 0, "线性规划", "green"),
    (2, 1.5, "SAT", "red"),
    (3, -0.5, "TSP", "red"),
    (2.5, 0.5, "图着色", "red"),
    (5, 0, "停机问题", "purple")
]

for x, y, name, color in problems:
    ax.text(x, y, name, color=color, fontsize=10, ha='center', 
            bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.3'))

# P=NP问题标注
if np.random.random() > 0.5:  # 随机选择一种情况
    ax.text(0, 0, "P=NP?", fontsize=16, ha='center', va='center',
            bbox=dict(facecolor='yellow', alpha=0.5, boxstyle='round,pad=0.5'))
else:
    # 绘制P≠NP的情况
    ax.text(0, 0, "P≠NP?", fontsize=16, ha='center', va='center',
            bbox=dict(facecolor='yellow', alpha=0.5, boxstyle='round,pad=0.5'))

# 添加图例
ax.legend(loc='upper right')

# 添加标题和说明
plt.title('P vs NP问题的可视化', fontsize=18)
ax.text(-5.8, -4.5, "注：如果P=NP，则绿色区域将与蓝色区域重合", fontsize=10)

# 移除坐标轴
ax.set_xticks([])
ax.set_yticks([])
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['bottom'].set_visible(False)
ax.spines['left'].set_visible(False)

# 保存图片
plt.savefig('img/pnp_visualization.png', dpi=300, bbox_inches='tight')
plt.close()

print("P vs NP问题可视化图已保存为'img/pnp_visualization.png'")