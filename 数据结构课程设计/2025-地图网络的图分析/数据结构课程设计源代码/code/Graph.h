#pragma once
#include <vector>
#include <string>
#include <limits>
#include "Vertex.h"
#include "Edge.h"
#include "RBTree.h"


class Graph {
public:
    Graph();
    ~Graph();
    Graph(const Graph&) = delete;
    Graph& operator=(const Graph&) = delete;

    Graph(Graph&&) noexcept = default;
    Graph& operator=(Graph&&) noexcept = default;

    // 顶点操作
    int addVertex(const std::string& name, double x = 0.0, double y = 0.0);
    bool removeVertex(int id);

    // 边的操作（无向图）
    bool addEdge(int u, int v, double weight);
    bool removeEdge(int u, int v);

    // 查询操作
    bool hasVertex(int id) const;
    bool hasEdge(int u, int v) const;
    double getWeight(int u, int v) const;

    // 获取顶点的所有邻接边
    const std::vector<Edge>& adjEdges(int u) const;
    
    // 获取顶点的所有邻接顶点
    std::vector<int> neighbors(int u) const;

    int vertexCount() const;
    const std::vector<Vertex>& vertices() const;

    void clear();

private:
    std::vector<Vertex> mVertices;
    std::vector<std::vector<Edge>> mAdjList;

    // 边索引：key = encodeEdge(u, v) -> 权重
    RBTree<long long, double> mEdgeIndex;

private:
    bool isValidId(int id) const;
    long long encodeEdge(int u, int v) const;
};
