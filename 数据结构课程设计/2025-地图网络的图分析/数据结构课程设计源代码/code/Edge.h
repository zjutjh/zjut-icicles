#pragma once

// 边结构体：表示图中两个顶点之间的有向边
struct Edge {
    int from = -1;           // 起点顶点ID
    int to = -1;             // 终点顶点ID
    double weight = 0.0;     // 边的权重（距离）

    Edge() = default;
    Edge(int from, int to, double weight)
        : from(from), to(to), weight(weight) {}
};
