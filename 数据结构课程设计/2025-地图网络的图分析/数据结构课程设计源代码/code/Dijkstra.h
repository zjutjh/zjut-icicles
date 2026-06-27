#pragma once
#include "Graph.h"
#include "Path.h"
#include "RBTree.h"

// 禁用集合类型（给Yen算法使用）
using BannedNodeSet = RBTree<int, bool>;
using BannedEdgeSet = RBTree<long long, bool>;

// 迪杰斯特拉最短路算法（支持禁用节点/边）
// 参数：图、源点、目标点、禁用节点集合、禁用边集合、返回的路径
// 返回：找到路径返回true，并写入result；否则false（result会被清空）
bool dijkstraShortestPath(
    const Graph& graph,
    int source,
    int target,
    const BannedNodeSet* bannedNodes,
    const BannedEdgeSet* bannedEdges,
    Path& result
);
