#pragma once
#include "Graph.h"
#include "Path.h"

// Yen算法（K=2）：求出最短路径和次短路径（非严格次短，允许等长）
// 参数：图、源点、目标点、返回的最短路径、返回的次短路径
// 返回：最短路径不存在时返回false
//      最短路径存在时返回true（secondShortestPath可能为空表示无次短路）
bool yenK2(
    const Graph& graph,
    int source,
    int target,
    Path& shortestPath,
    Path& secondShortestPath
);
