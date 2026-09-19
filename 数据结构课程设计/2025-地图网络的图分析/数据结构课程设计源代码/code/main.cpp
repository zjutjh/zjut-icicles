#include <iostream>
#include "Graph.h"
#include "Yen.h"
#include "FileIO.h"

// 打印路径信息：显示路径上的节点和总代价
void printPath(const Path& path) {
    if (path.nodes.empty()) {
        std::cout << "（空）\n";
        return;
    }
    for (std::size_t i = 0; i < path.nodes.size(); i++) {
        std::cout << path.nodes[i];
        if (i + 1 < path.nodes.size()) std::cout << " -> ";
    }
    std::cout << "   权重 = " << path.cost << "\n";
}

// 主函数：演示Yen算法求最短路和次短路
int main() {
    Graph graph;
    
    /*
        构造一个菱形图用于演示：
        
            (B)
           /   \
        (A)     (D)
           \   /
            (C)
        
        所有边的权重都为1
        从A到D有两条等长路径：
        A->B->D  (代价=2)
        A->C->D  (代价=2)
    */
    
    int v0 = graph.addVertex("A");
    int v1 = graph.addVertex("B");
    int v2 = graph.addVertex("C");
    int v3 = graph.addVertex("D");

    graph.addEdge(v0, v1, 1.0);
    graph.addEdge(v1, v3, 1.0);
    graph.addEdge(v0, v2, 1.0);
    graph.addEdge(v2, v3, 1.0);

    Path shortest;
    Path secondShortest;

    bool ok = yenK2(graph, v0, v3, shortest, secondShortest);

    if (!ok) {
        std::cout << "未找到从 " << v0 << " 到 " << v3 << " 的路径\n";
        return 0;
    }

    std::cout << "最短路径:\n";
    printPath(shortest);

    std::cout << "次短路径:\n";
    printPath(secondShortest);
    
    // 测试文件保存和加载功能
    saveMapFile("test.map", graph);
    
    // 从文件重新加载图数据
    Graph g2;
    loadMapFile("test.map", g2);

    // 在重新加载的图上运行Yen算法
    Path p1, p2;
    yenK2(g2, 0, 3, p1, p2);

    /*
        期望输出（两条路径的顺序可能互换，属于正常情况）：
        
        最短路径:
        0 -> 1 -> 3   权重 = 2
        
        次短路径:
        0 -> 2 -> 3   权重 = 2
    */

    return 0;
}
