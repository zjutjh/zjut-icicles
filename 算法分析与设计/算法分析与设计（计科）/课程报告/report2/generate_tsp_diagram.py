import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei']  # 使用黑体
plt.rcParams['axes.unicode_minus'] = False  # 正确显示负号

# 创建图形
fig, axs = plt.subplots(1, 2, figsize=(14, 6))

# 第一个子图：TSP问题的城市图
ax1 = axs[0]

# 创建一个完全图
n_cities = 5
G = nx.complete_graph(n_cities)

# 为每个城市分配随机位置
np.random.seed(42)  # 固定随机种子以获得可重复的结果
pos = {i: (np.random.random(), np.random.random()) for i in range(n_cities)}

# 为边分配距离（基于城市间的欧氏距离）
for u, v in G.edges():
    G[u][v]['weight'] = np.sqrt((pos[u][0] - pos[v][0])**2 + (pos[u][1] - pos[v][1])**2)

# 绘制节点
nx.draw_networkx_nodes(G, pos, node_size=700, 
                      node_color='skyblue', ax=ax1)

# 绘制边（所有可能的路径）
nx.draw_networkx_edges(G, pos, width=1, alpha=0.3, ax=ax1)

# 绘制节点标签（城市名称）
city_labels = {i: f"城市{i+1}" for i in range(n_cities)}
nx.draw_networkx_labels(G, pos, labels=city_labels, font_size=12, font_family='sans-serif', ax=ax1)

# 绘制边标签（距离）
edge_labels = {(u, v): f"{G[u][v]['weight']:.2f}" for u, v in G.edges()}
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=8, ax=ax1)

# 第二个子图：TSP问题的最优解示例
ax2 = axs[1]

# 创建一个包含最优路径的图
optimal_path = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 0)]
G_optimal = nx.Graph()
G_optimal.add_edges_from(optimal_path)

# 绘制节点
nx.draw_networkx_nodes(G_optimal, pos, node_size=700, 
                      node_color='skyblue', ax=ax2)

# 绘制最优路径的边
nx.draw_networkx_edges(G_optimal, pos, width=3, 
                      edge_color='red', ax=ax2)

# 绘制节点标签
nx.draw_networkx_labels(G_optimal, pos, labels=city_labels, 
                       font_size=12, font_family='sans-serif', ax=ax2)

# 计算最优路径的总长度
total_distance = sum(G[u][v]['weight'] for u, v in optimal_path)

# 添加路径长度信息
ax2.text(0.5, -0.1, f"最优路径总长度: {total_distance:.2f}", 
        transform=ax2.transAxes, fontsize=14, 
        ha='center', va='center')

# 设置标题
ax1.set_title('TSP问题：所有可能的城市间连接', fontsize=14)
ax2.set_title('TSP问题：一个可能的最优解', fontsize=14)

# 关闭坐标轴
ax1.axis('off')
ax2.axis('off')

# 调整布局
plt.tight_layout()

# 保存图片
plt.savefig('./img/tsp_example.png', dpi=300, bbox_inches='tight')
plt.close()

# 创建第二个图：TSP与SAT的关系
fig, ax = plt.subplots(figsize=(10, 6))

# 创建流程图
steps = [
    "TSP问题\n(旅行商问题)",
    "决策版本TSP\n(路径长度≤L?)",
    "SAT问题\n(布尔可满足性问题)",
    "其他NP完全问题"
]

positions = [(0, 0), (0, -2), (3, -2), (3, 0)]
box_width = 2.5
box_height = 0.8

# 绘制框和文本
for i, (step, pos) in enumerate(zip(steps, positions)):
    rect = plt.Rectangle((pos[0] - box_width/2, pos[1] - box_height/2), 
                        box_width, box_height, 
                        facecolor='lightblue', edgecolor='blue', alpha=0.7)
    ax.add_patch(rect)
    ax.text(pos[0], pos[1], step, ha='center', va='center', fontsize=12)

# 绘制箭头
arrows = [
    ((0, -0.4), (0, -1.6), "转化为决策问题"),
    ((0.4, -2), (2.6, -2), "多项式时间规约"),
    ((3, -1.6), (3, -0.4), "多项式时间规约"),
    ((2.6, 0), (0.4, 0), "多项式时间规约")
]

for start, end, label in arrows:
    ax.annotate("", xy=end, xytext=start,
               arrowprops=dict(facecolor='black', shrink=0.05, 
                              width=1.5, headwidth=8))
    # 计算箭头中点位置
    mid_x = (start[0] + end[0]) / 2
    mid_y = (start[1] + end[1]) / 2
    # 添加标签
    ax.text(mid_x, mid_y, label, ha='center', va='center', 
           fontsize=10, bbox=dict(facecolor='white', alpha=0.7, boxstyle='round'))

# 设置标题
ax.set_title('TSP问题与SAT问题的关系', fontsize=16)

# 设置坐标轴范围
ax.set_xlim(-2, 5)
ax.set_ylim(-3, 1)

# 关闭坐标轴
ax.axis('off')

# 保存图片
plt.savefig('./img/tsp_sat_relation.png', dpi=300, bbox_inches='tight')
plt.close()

print("TSP问题示例图和TSP-SAT关系图已生成到 ./img/目录") 