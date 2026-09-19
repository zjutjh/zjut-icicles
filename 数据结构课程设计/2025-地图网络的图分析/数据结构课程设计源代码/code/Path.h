#pragma once
#include <vector>
#include <string>

// 路径结构体：表示一条从源到目标的路径
struct Path {
    std::vector<int> nodes;  // 路径上的节点序列
    double cost = 0.0;       // 路径的总代价（权重之和）

    bool empty() const { return nodes.empty(); }
    std::string toString() const;
};
