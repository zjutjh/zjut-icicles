import matplotlib.pyplot as plt
import networkx as nx
import numpy as np

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False    # 用来正常显示负号

# 创建图
def create_mst_example():
    # 创建一个带权无向图
    G = nx.Graph()
    
    # 添加节点
    G.add_nodes_from(['A', 'B', 'C', 'D', 'E', 'F'])
    
    # 添加带权边
    edges = [
        ('A', 'B', 4), ('A', 'C', 2), 
        ('B', 'C', 1), ('B', 'D', 5),
        ('C', 'D', 8), ('C', 'E', 10),
        ('D', 'E', 2), ('D', 'F', 6),
        ('E', 'F', 3)
    ]
    G.add_weighted_edges_from(edges)
    
    # 计算最小生成树
    T = nx.minimum_spanning_tree(G, weight='weight')
    
    # 绘制图形
    plt.figure(figsize=(12, 5))
    
    # 原始图
    plt.subplot(1, 2, 1)
    pos = nx.spring_layout(G, seed=42)  # 固定布局，使得两个图的节点位置相同
    
    nx.draw_networkx_nodes(G, pos, node_color='lightblue', node_size=500)
    nx.draw_networkx_labels(G, pos, font_weight='bold')
    
    edge_labels = {(u, v): d['weight'] for u, v, d in G.edges(data=True)}
    nx.draw_networkx_edges(G, pos, width=1.5)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=12)
    
    plt.title('原始带权图', fontsize=16)
    plt.axis('off')
    
    # 最小生成树
    plt.subplot(1, 2, 2)
    
    # 绘制所有点（包括原始图中的所有点）
    nx.draw_networkx_nodes(G, pos, node_color='lightblue', node_size=500)
    nx.draw_networkx_labels(G, pos, font_weight='bold')
    
    # 只绘制MST中的边
    edge_labels_mst = {(u, v): d['weight'] for u, v, d in T.edges(data=True)}
    nx.draw_networkx_edges(T, pos, width=2.5, edge_color='red')
    nx.draw_networkx_edge_labels(T, pos, edge_labels=edge_labels_mst, font_size=12)
    
    plt.title('最小生成树', fontsize=16)
    plt.axis('off')
    
    plt.tight_layout()
    plt.savefig('img/mst_example.png', dpi=300, bbox_inches='tight')
    plt.close()



if __name__ == "__main__":
    # 确保img目录存在
    import os
    if not os.path.exists('img'):
        os.makedirs('img')
    
    # 生成图像
    create_mst_example()
    
    print("图像已生成到img目录。") 