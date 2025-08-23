import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'Microsoft YaHei', 'sans-serif']  # 增加备用字体
plt.rcParams['axes.unicode_minus'] = False  # 正确显示负号

# 创建有向图
G = nx.DiGraph()

# 添加节点（NP完全问题）
problems = [
    "SAT", "3-SAT", "独立集", "顶点覆盖", "图着色", 
    "哈密顿回路", "TSP", "子集和", "分割问题", "最大割"
]

# 添加节点
G.add_nodes_from(problems)

# 添加边（归约关系）
reductions = [
    ("SAT", "3-SAT"),
    ("3-SAT", "独立集"),
    ("3-SAT", "顶点覆盖"),
    ("3-SAT", "图着色"),
    ("独立集", "顶点覆盖"),
    ("顶点覆盖", "哈密顿回路"),
    ("哈密顿回路", "TSP"),
    ("3-SAT", "子集和"),
    ("子集和", "分割问题"),
    ("3-SAT", "最大割")
]

G.add_edges_from(reductions)

# 创建图形
plt.figure(figsize=(12, 10))

# 设置布局
pos = nx.spring_layout(G, seed=42, k=0.5)  # k控制节点间距

# 绘制节点
nx.draw_networkx_nodes(G, pos, 
                      node_color='lightblue', 
                      node_size=2000, 
                      alpha=0.8)

# 绘制边
nx.draw_networkx_edges(G, pos, 
                      edge_color='gray',
                      width=1.5, 
                      arrowsize=20,
                      alpha=0.7)

# 添加节点标签
nx.draw_networkx_labels(G, pos, 
                       font_size=12, 
                       font_family='sans-serif',
                       font_weight='bold')

# 添加边标签（归约类型）
edge_labels = {
    ("SAT", "3-SAT"): "多项式变换",
    ("3-SAT", "独立集"): "多项式变换",
    ("3-SAT", "顶点覆盖"): "多项式变换",
    ("3-SAT", "图着色"): "多项式变换",
    ("独立集", "顶点覆盖"): "补图变换",
    ("顶点覆盖", "哈密顿回路"): "多项式变换",
    ("哈密顿回路", "TSP"): "加权变换",
    ("3-SAT", "子集和"): "多项式变换",
    ("子集和", "分割问题"): "直接变换",
    ("3-SAT", "最大割"): "多项式变换"
}

nx.draw_networkx_edge_labels(G, pos, 
                           edge_labels=edge_labels, 
                           font_size=8,
                           font_color='red',
                           font_family='sans-serif',
                           alpha=0.7)

# 添加标题
plt.title('NP完全问题之间的归约关系', fontsize=18)

# 添加说明
plt.text(0.5, -0.1, 
         "注：箭头A→B表示问题A可以归约到问题B，即如果B可以在多项式时间内解决，则A也可以",
         horizontalalignment='center',
         fontsize=12,
         transform=plt.gca().transAxes)

# 移除坐标轴
plt.axis('off')

# 保存图片
plt.savefig('img/np_reductions.png', dpi=300, bbox_inches='tight')
plt.close()

print("NP完全问题归约关系图已保存为'img/np_reductions.png'") 