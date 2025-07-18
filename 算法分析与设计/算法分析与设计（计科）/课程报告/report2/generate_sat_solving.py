import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import networkx as nx
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'Microsoft YaHei', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False  # 正确显示负号

# 创建图形
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 8))

# 第一部分：SAT问题的表示
ax1.set_title('SAT问题的表示与转化', fontsize=16)
ax1.axis('off')

# 添加CNF公式
formula = r"$(x_1 \vee \neg x_2 \vee x_3) \wedge (\neg x_1 \vee x_2 \vee \neg x_4) \wedge (x_2 \vee x_3 \vee x_4)$"
ax1.text(0.5, 0.9, "CNF公式示例:", fontsize=14, ha='center')
ax1.text(0.5, 0.8, formula, fontsize=14, ha='center')

# 添加子句解释
ax1.text(0.1, 0.7, "子句1: $(x_1 \vee \neg x_2 \vee x_3)$", fontsize=12)
ax1.text(0.1, 0.65, "子句2: $(\neg x_1 \vee x_2 \vee \neg x_4)$", fontsize=12)
ax1.text(0.1, 0.6, "子句3: $(x_2 \vee x_3 \vee x_4)$", fontsize=12)

# 绘制子句图
G = nx.Graph()
positions = {
    'x1': (0.2, 0.4),
    'x2': (0.4, 0.4),
    'x3': (0.6, 0.4),
    'x4': (0.8, 0.4),
    'c1': (0.3, 0.2),
    'c2': (0.5, 0.2),
    'c3': (0.7, 0.2)
}

# 添加节点
G.add_node('x1', pos=positions['x1'])
G.add_node('x2', pos=positions['x2'])
G.add_node('x3', pos=positions['x3'])
G.add_node('x4', pos=positions['x4'])
G.add_node('c1', pos=positions['c1'])
G.add_node('c2', pos=positions['c2'])
G.add_node('c3', pos=positions['c3'])

# 添加边
G.add_edge('x1', 'c1', color='green')  # 正文字
G.add_edge('x2', 'c1', color='red')    # 负文字
G.add_edge('x3', 'c1', color='green')  # 正文字
G.add_edge('x1', 'c2', color='red')    # 负文字
G.add_edge('x2', 'c2', color='green')  # 正文字
G.add_edge('x4', 'c2', color='red')    # 负文字
G.add_edge('x2', 'c3', color='green')  # 正文字
G.add_edge('x3', 'c3', color='green')  # 正文字
G.add_edge('x4', 'c3', color='green')  # 正文字

# 获取边的颜色
edge_colors = [G[u][v]['color'] for u, v in G.edges()]

# 绘制图
pos = nx.get_node_attributes(G, 'pos')
nx.draw_networkx_nodes(G, pos, nodelist=['x1', 'x2', 'x3', 'x4'], 
                      node_color='lightblue', node_size=500, ax=ax1)
nx.draw_networkx_nodes(G, pos, nodelist=['c1', 'c2', 'c3'], 
                      node_color='lightgreen', node_size=500, ax=ax1)
nx.draw_networkx_edges(G, pos, edge_color=edge_colors, width=2, ax=ax1)
nx.draw_networkx_labels(G, pos, {'x1': '$x_1$', 'x2': '$x_2$', 'x3': '$x_3$', 'x4': '$x_4$',
                               'c1': '子句1', 'c2': '子句2', 'c3': '子句3'}, font_size=12, ax=ax1)

# 添加图例
ax1.text(0.2, 0.1, "绿色边: 正文字", color='green', fontsize=10)
ax1.text(0.6, 0.1, "红色边: 负文字", color='red', fontsize=10)

# 第二部分：SAT求解过程（DPLL算法决策树）
ax2.set_title('SAT求解过程 (DPLL算法)', fontsize=16)
ax2.axis('off')

# 创建决策树
tree_pos = {
    'root': (0.5, 0.9),
    'x1_0': (0.3, 0.7),
    'x1_1': (0.7, 0.7),
    'x2_00': (0.2, 0.5),
    'x2_01': (0.4, 0.5),
    'x2_10': (0.6, 0.5),
    'x2_11': (0.8, 0.5),
    'x3_000': (0.15, 0.3),
    'x3_001': (0.25, 0.3),
    'x3_010': (0.35, 0.3),
    'x3_011': (0.45, 0.3),
    'x3_100': (0.55, 0.3),
    'x3_101': (0.65, 0.3),
    'x3_110': (0.75, 0.3),
    'x3_111': (0.85, 0.3),
}

# 绘制节点
for node, pos in tree_pos.items():
    if node == 'root':
        label = '$x_1$'
        color = 'lightblue'
    elif node.startswith('x1'):
        label = '$x_2$'
        color = 'lightblue'
    elif node.startswith('x2'):
        label = '$x_3$'
        color = 'lightblue'
    elif node.startswith('x3'):
        # 判断是否为满足解
        if node in ['x3_001', 'x3_011', 'x3_101', 'x3_111']:
            label = '满足'
            color = 'lightgreen'
        else:
            label = '不满足'
            color = 'lightcoral'
    
    circle = plt.Circle(pos, 0.05, color=color, ec='black')
    ax2.add_patch(circle)
    ax2.text(pos[0], pos[1], label, ha='center', va='center', fontsize=10)

# 绘制边
def draw_edge(ax, start, end, label):
    ax.annotate("", xy=end, xytext=start,
               arrowprops=dict(arrowstyle="->", lw=1.5))
    # 添加标签
    mid_x = (start[0] + end[0]) / 2
    mid_y = (start[1] + end[1]) / 2
    ax.text(mid_x, mid_y, label, ha='center', va='center', fontsize=10,
           bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

# 第一层边
draw_edge(ax2, tree_pos['root'], tree_pos['x1_0'], "$x_1=0$")
draw_edge(ax2, tree_pos['root'], tree_pos['x1_1'], "$x_1=1$")

# 第二层边
draw_edge(ax2, tree_pos['x1_0'], tree_pos['x2_00'], "$x_2=0$")
draw_edge(ax2, tree_pos['x1_0'], tree_pos['x2_01'], "$x_2=1$")
draw_edge(ax2, tree_pos['x1_1'], tree_pos['x2_10'], "$x_2=0$")
draw_edge(ax2, tree_pos['x1_1'], tree_pos['x2_11'], "$x_2=1$")

# 第三层边
draw_edge(ax2, tree_pos['x2_00'], tree_pos['x3_000'], "$x_3=0$")
draw_edge(ax2, tree_pos['x2_00'], tree_pos['x3_001'], "$x_3=1$")
draw_edge(ax2, tree_pos['x2_01'], tree_pos['x3_010'], "$x_3=0$")
draw_edge(ax2, tree_pos['x2_01'], tree_pos['x3_011'], "$x_3=1$")
draw_edge(ax2, tree_pos['x2_10'], tree_pos['x3_100'], "$x_3=0$")
draw_edge(ax2, tree_pos['x2_10'], tree_pos['x3_101'], "$x_3=1$")
draw_edge(ax2, tree_pos['x2_11'], tree_pos['x3_110'], "$x_3=0$")
draw_edge(ax2, tree_pos['x2_11'], tree_pos['x3_111'], "$x_3=1$")

# 添加一个满足解的示例
ax2.text(0.5, 0.15, "满足解示例: $x_1=1, x_2=1, x_3=1, x_4=1$", fontsize=12, ha='center',
        bbox=dict(facecolor='lightgreen', alpha=0.5, boxstyle='round,pad=0.5'))

# 添加DPLL算法的主要步骤
steps = [
    "1. 单元子句传播",
    "2. 纯文字消除",
    "3. 变量选择",
    "4. 递归搜索",
    "5. 回溯"
]

for i, step in enumerate(steps):
    ax2.text(0.1, 0.05 - i * 0.03, step, fontsize=10, ha='left')

# 调整子图之间的间距
plt.tight_layout()

# 保存图片
plt.savefig('img/sat_solving_process.png', dpi=300, bbox_inches='tight')
plt.close()

print("SAT问题求解过程图已保存为'img/sat_solving_process.png'") 