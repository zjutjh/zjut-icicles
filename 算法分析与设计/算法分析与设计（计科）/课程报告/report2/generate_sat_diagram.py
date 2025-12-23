import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import networkx as nx
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端

# 创建图形
fig, axs = plt.subplots(1, 2, figsize=(12, 5))

# 第一个子图：SAT问题的CNF表示
ax1 = axs[0]

# 绘制CNF公式
formula = "(x1 OR NOT x2) AND (x2 OR x3) AND (NOT x1 OR NOT x3)"
ax1.text(0.5, 0.7, formula, fontsize=12, ha='center', va='center')

# 绘制变量赋值表格
table_data = [
    ['Variable', 'Value'],
    ['x1', 'True'],
    ['x2', 'True'],
    ['x3', 'False']
]

cell_colors = [['lightgray', 'lightgray'], ['white', 'lightgreen'], 
               ['white', 'lightgreen'], ['white', 'lightgreen']]

table = ax1.table(cellText=table_data, cellColours=cell_colors,
                  cellLoc='center', loc='center', bbox=[0.3, 0.2, 0.4, 0.4])
table.auto_set_font_size(False)
table.set_fontsize(12)
table.scale(1, 1.5)

# 添加验证结果
ax1.text(0.5, 0.1, "Result: Satisfiable", fontsize=14, ha='center', va='center', color='green')

# 第二个子图：SAT问题的图表示
ax2 = axs[1]

# 创建一个有向图
G = nx.DiGraph()

# 添加节点
G.add_node("x1", pos=(0, 2))
G.add_node("-x1", pos=(2, 2))
G.add_node("x2", pos=(0, 1))
G.add_node("-x2", pos=(2, 1))
G.add_node("x3", pos=(0, 0))
G.add_node("-x3", pos=(2, 0))

# 添加边（表示子句）
G.add_edge("x1", "-x2", color='blue', label="C1")
G.add_edge("x2", "x3", color='red', label="C2")
G.add_edge("-x1", "-x3", color='green', label="C3")

# 获取节点位置
pos = nx.get_node_attributes(G, 'pos')

# 绘制节点
nx.draw_networkx_nodes(G, pos, node_size=1500, 
                      node_color='lightblue', ax=ax2)

# 绘制边
edge_colors = [G[u][v]['color'] for u, v in G.edges()]
nx.draw_networkx_edges(G, pos, width=2, edge_color=edge_colors, 
                      arrowstyle='->', arrowsize=15, ax=ax2)

# 绘制节点标签
nx.draw_networkx_labels(G, pos, font_size=14, font_family='sans-serif', ax=ax2)

# 绘制边标签
edge_labels = {(u, v): G[u][v]['label'] for u, v in G.edges()}
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=12, ax=ax2)

# 设置标题
ax1.set_title('SAT Problem: CNF Representation', fontsize=14)
ax2.set_title('SAT Problem: Graph Representation', fontsize=14)

# 关闭坐标轴
ax1.axis('off')
ax2.axis('off')

# 保存图片
plt.savefig('./img/sat_example.png', dpi=300, bbox_inches='tight')
plt.close()

print("SAT问题示例图已生成到 ./img/sat_example.png") 