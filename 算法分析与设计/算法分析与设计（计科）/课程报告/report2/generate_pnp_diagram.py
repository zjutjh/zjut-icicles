import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from matplotlib.path import Path
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'Microsoft YaHei', 'sans-serif']  # 增加备用字体
plt.rcParams['axes.unicode_minus'] = False  # 正确显示负号

# 创建图形和坐标轴
fig, ax = plt.subplots(figsize=(8, 6))

# 创建NP类椭圆
np_ellipse = patches.Ellipse((0.5, 0.5), 0.8, 0.7, 
                             edgecolor='blue', facecolor='lightblue', 
                             alpha=0.5, label='NP类')

# 创建P类椭圆
p_ellipse = patches.Ellipse((0.3, 0.5), 0.4, 0.5, 
                            edgecolor='red', facecolor='lightcoral', 
                            alpha=0.5, label='P类')

# 创建NP完全类区域（NP类的右侧部分）
npc_ellipse = patches.Ellipse((0.75, 0.5), 0.3, 0.5, 
                             edgecolor='green', facecolor='lightgreen', 
                             alpha=0.5, label='NP完全类')

# 添加椭圆到图表
ax.add_patch(np_ellipse)
ax.add_patch(p_ellipse)
ax.add_patch(npc_ellipse)

# 添加文本标签
ax.text(0.3, 0.5, 'P', fontsize=20, ha='center', va='center')
ax.text(0.75, 0.5, 'NP完全', fontsize=16, ha='center', va='center')
ax.text(0.5, 0.8, 'NP', fontsize=20, ha='center', va='center')

# 添加问号表示P=NP?的未解决问题
ax.text(0.5, 0.3, '?', fontsize=30, ha='center', va='center', color='purple')

# 添加注释
ax.annotate('P=NP?', xy=(0.5, 0.2), xytext=(0.5, 0.1),
            fontsize=14, ha='center', va='center',
            arrowprops=dict(facecolor='black', shrink=0.05, width=1.5))

# 添加一些示例问题
ax.text(0.2, 0.6, '最短路径', fontsize=8, ha='center', va='center')
ax.text(0.3, 0.4, '线性规划', fontsize=8, ha='center', va='center')
ax.text(0.75, 0.6, 'SAT问题', fontsize=8, ha='center', va='center')
ax.text(0.75, 0.4, 'TSP问题', fontsize=8, ha='center', va='center')

# 设置坐标轴范围
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)

# 隐藏坐标轴
ax.axis('off')

# 添加标题
ax.set_title('P、NP和NP完全问题的关系', fontsize=16)

# 添加图例
ax.legend(handles=[p_ellipse, np_ellipse, npc_ellipse], 
          loc='upper center', bbox_to_anchor=(0.5, -0.05), 
          ncol=3, frameon=False)

# 保存图片
plt.savefig('./img/pnp_relation.png', dpi=300, bbox_inches='tight')
plt.close()

print("P、NP和NP完全问题关系图已生成到 ./img/pnp_relation.png") 