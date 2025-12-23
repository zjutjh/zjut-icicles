import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import networkx as nx
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端

# 设置字体支持
plt.rcParams['font.sans-serif'] = ['SimSun', 'SimHei', 'Microsoft YaHei', 'DejaVu Sans', 'Arial', 'sans-serif']
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['axes.unicode_minus'] = False  # 正确显示负号

# 创建图形
fig, ax = plt.subplots(figsize=(12, 10), dpi=100)

# 设置背景颜色
ax.set_facecolor('#f8f8f8')

# 创建TSP示例图
def create_tsp_example():
    G = nx.DiGraph()
    
    # 添加节点
    cities = {
        'A': (0.2, 0.7),
        'B': (0.3, 0.9),
        'C': (0.5, 0.8),
        'D': (0.7, 0.9),
        'E': (0.8, 0.7)
    }
    
    for city, pos in cities.items():
        G.add_node(city, pos=pos)
    
    # 添加边
    edges = [
        ('A', 'B', 5),
        ('A', 'C', 7),
        ('A', 'D', 9),
        ('A', 'E', 8),
        ('B', 'A', 5),
        ('B', 'C', 4),
        ('B', 'D', 8),
        ('B', 'E', 10),
        ('C', 'A', 7),
        ('C', 'B', 4),
        ('C', 'D', 6),
        ('C', 'E', 5),
        ('D', 'A', 9),
        ('D', 'B', 8),
        ('D', 'C', 6),
        ('D', 'E', 3),
        ('E', 'A', 8),
        ('E', 'B', 10),
        ('E', 'C', 5),
        ('E', 'D', 3)
    ]
    
    for u, v, w in edges:
        G.add_edge(u, v, weight=w)
    
    return G, cities

# 创建SAT示例 - 使用ASCII字符
def create_sat_example():
    clauses = [
        "(x1 OR x2 OR NOT x3)",
        "(NOT x1 OR NOT x2 OR x4)",
        "(x2 OR NOT x3 OR NOT x4)",
        "(x1 OR x3 OR x4)"
    ]
    return clauses

# 绘制TSP示例
def draw_tsp_example(ax, G, cities, pos_x=0.2, pos_y=0.7, scale=0.15):
    # 调整节点位置
    pos = {}
    for city, (x, y) in cities.items():
        pos[city] = (pos_x + (x - 0.5) * scale, pos_y + (y - 0.8) * scale)
    
    # 绘制节点
    nx.draw_networkx_nodes(G, pos, ax=ax, node_size=300, node_color='skyblue', edgecolors='black')
    
    # 绘制边和权重
    edge_labels = {(u, v): d['weight'] for u, v, d in G.edges(data=True)}
    nx.draw_networkx_edges(G, pos, ax=ax, width=1.0, alpha=0.7, arrows=True, arrowsize=15)
    nx.draw_networkx_labels(G, pos, ax=ax, font_size=10, font_weight='bold')
    
    # 绘制权重标签
    for (u, v, d) in G.edges(data=True):
        x1, y1 = pos[u]
        x2, y2 = pos[v]
        x = (x1 + x2) / 2
        y = (y1 + y2) / 2
        # 稍微偏移标签位置，避免与边重叠
        offset = 0.005
        if x1 < x2:
            x += offset
        else:
            x -= offset
        if y1 < y2:
            y += offset
        else:
            y -= offset
        ax.text(x, y, str(d['weight']), fontsize=8, ha='center', va='center',
                bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', pad=1))
    
    # 添加标题
    ax.text(pos_x, pos_y + 0.1, "TSP问题示例", fontsize=12, ha='center', fontweight='bold')
    
    # 添加说明
    ax.text(pos_x, pos_y - 0.1, "目标：找出访问所有城市\n并返回起点的最短路径", 
            fontsize=10, ha='center', va='center',
            bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.3'))

# 绘制SAT示例
def draw_sat_example(ax, clauses, pos_x=0.8, pos_y=0.7, width=0.25, height=0.15):
    # 绘制背景框
    rect = patches.Rectangle((pos_x - width/2, pos_y - height), width, height*2, 
                           facecolor='#E6F5D0', edgecolor='black', alpha=0.8, linewidth=1)
    ax.add_patch(rect)
    
    # 添加标题
    ax.text(pos_x, pos_y + height, "SAT问题示例", fontsize=12, ha='center', fontweight='bold')
    
    # 添加公式
    formula_text = " AND ".join(clauses)
    ax.text(pos_x, pos_y, formula_text, fontsize=10, ha='center', va='center',
            bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.3'))
    
    # 添加说明
    ax.text(pos_x, pos_y - height/2, "目标：找出使公式为真的变量赋值", 
            fontsize=10, ha='center', va='center',
            bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.3'))

# 绘制归约过程
def draw_reduction(ax, start_x=0.35, start_y=0.5, end_x=0.65, end_y=0.5):
    # 绘制箭头
    ax.annotate("", xy=(end_x, end_y), xytext=(start_x, end_y),
               arrowprops=dict(arrowstyle="->", lw=2, color='red'))
    
    # 添加归约标签
    ax.text((start_x + end_x) / 2, end_y + 0.02, "多项式时间归约", 
            fontsize=12, ha='center', va='center', color='red', fontweight='bold')
    
    # 添加变量设计说明 - 使用ASCII字符
    variables_text = "变量设计:\n" + \
                    "* x[i,j,p]: 城市i在位置p访问城市j\n" + \
                    "* 位置唯一性约束\n" + \
                    "* 访问唯一性约束\n" + \
                    "* 路径连续性约束\n" + \
                    "* 路径长度约束"
    
    ax.text((start_x + end_x) / 2, end_y - 0.1, variables_text, 
            fontsize=10, ha='center', va='center',
            bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.3'))

# 绘制理论意义
def draw_theoretical_significance(ax, pos_x=0.5, pos_y=0.2, width=0.7, height=0.15):
    # 绘制背景框
    rect = patches.Rectangle((pos_x - width/2, pos_y - height), width, height*2, 
                           facecolor='#FFE6CC', edgecolor='black', alpha=0.8, linewidth=1)
    ax.add_patch(rect)
    
    # 添加标题
    ax.text(pos_x, pos_y + height, "理论意义", fontsize=14, ha='center', fontweight='bold')
    
    # 添加要点 - 使用ASCII字符
    points = [
        "* 证明了TSP和SAT都是NP完全问题",
        "* 归约建立了问题之间的桥梁",
        "* 任何一个问题的多项式解法都意味着P=NP",
        "* 为算法设计提供了不同视角",
        "* 启发了混合求解策略的开发"
    ]
    
    point_text = "\n".join(points)
    ax.text(pos_x, pos_y - height/2 + 0.02, point_text, 
            fontsize=10, ha='center', va='center',
            bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.3'))

# 绘制TSP和SAT的关系
G, cities = create_tsp_example()
clauses = create_sat_example()

draw_tsp_example(ax, G, cities)
draw_sat_example(ax, clauses)
draw_reduction(ax)
draw_theoretical_significance(ax)

# 添加标题
ax.text(0.5, 0.95, "TSP与SAT问题的关系", fontsize=18, ha='center', fontweight='bold')

# 设置坐标轴范围
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)

# 隐藏坐标轴
ax.axis('off')

# 保存图片
plt.savefig('img/tsp_sat_relation.png', dpi=300, bbox_inches='tight')
plt.close()

print("TSP与SAT问题关系图已保存为'img/tsp_sat_relation.png'") 