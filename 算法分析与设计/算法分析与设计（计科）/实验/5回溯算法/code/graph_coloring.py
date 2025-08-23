def graph_coloring(graph, m):
    # 顶点数
    n = len(graph)
    # 顶点颜色数组 (0表示未着色)
    colors = [0] * n
    
    def is_safe(vertex, color):
        # 检查是否可以将颜色color分配给顶点vertex
        for i in range(n):
            if graph[vertex][i] == 1 and colors[i] == color:
                return False
        return True
    
    def backtrack(vertex):
        # 所有顶点已着色，找到解
        if vertex == n:
            return True
        
        # 尝试为当前顶点分配不同的颜色
        for color in range(1, m + 1):
            if is_safe(vertex, color):
                # 给当前顶点着色
                colors[vertex] = color
                
                # 递归为下一个顶点着色
                if backtrack(vertex + 1):
                    return True
                
                # 回溯
                colors[vertex] = 0
        
        # 没有找到有效着色
        return False
    
    if not backtrack(0):
        print(f"没有找到使用{m}种颜色的着色方案")
        return None
    
    return colors

# 测试用例
if __name__ == "__main__":
    # 定义一个邻接矩阵表示的图，1表示两个顶点相连
    graph = [
        [0, 1, 1, 1],  # 顶点0与顶点1, 2, 3相连
        [1, 0, 1, 0],  # 顶点1与顶点0, 2相连
        [1, 1, 0, 1],  # 顶点2与顶点0, 1, 3相连
        [1, 0, 1, 0]   # 顶点3与顶点0, 2相连
    ]
    
    # 可用颜色数
    m = 3
    
    print("图的邻接矩阵表示:")
    for row in graph:
        print(" ".join(map(str, row)))
    print()
    
    # 求解图着色问题
    colors = graph_coloring(graph, m)
    
    if colors:
        print(f"使用{m}种颜色的着色方案:")
        for i in range(len(colors)):
            print(f"顶点 {i}: 颜色 {colors[i]}") 