#include "Dijkstra.h"
#include <queue>
#include <vector>
#include <limits>
#include <algorithm>

// 迪杰斯特拉算法的辅助函数和工具
namespace {
    // 将两个顶点ID编码为一个唯一的长整数，用于快速查找边
    long long encodeEdgeKey(int u, int v) {
        return static_cast<long long>(u) * 1000000LL + v;
    }

    // 检查某个节点是否在禁用集合中
    bool isBannedNode(int node, const BannedNodeSet* bannedNodes) {
        if (!bannedNodes) return false;
        return bannedNodes->contains(node);
    }

    // 检查某条边是否在禁用集合中
    bool isBannedEdge(int u, int v, const BannedEdgeSet* bannedEdges) {
        if (!bannedEdges) return false;
        return bannedEdges->contains(encodeEdgeKey(u, v));
    }

    // 保守校验：迪杰斯特拉算法只允许非负边权，且拒绝异常大值以避免溢出
    bool isValidWeight(double w) {
        // 检查是否为 NaN 或 Inf
        if (w != w || w == std::numeric_limits<double>::infinity() || 
            w == -std::numeric_limits<double>::infinity()) {
            return false;
        }
        if (w < 0.0) return false;
        // 这个阈值足够大：课设地图不可能需要更大；同时避免 dist + w 溢出成 inf
        if (w > 1e18) return false;
        return true;
    }
}

// 迪杰斯特拉最短路算法
// 参数：图，源点，目标点，禁用节点集合，禁用边集合，返回的路径
// 返回值：是否找到有效路径
bool dijkstraShortestPath(
    const Graph& graph,
    int source,
    int target,
    const BannedNodeSet* bannedNodes,
    const BannedEdgeSet* bannedEdges,
    Path& result
) {
    // 初始化结果路径
    result.nodes.clear();
    result.cost = 0.0;

    // 验证源点和目标点都存在于图中
    if (!graph.hasVertex(source) || !graph.hasVertex(target)) return false;

    // 特殊情况：源点等于目标点
    if (source == target) {
        if (isBannedNode(source, bannedNodes)) return false;
        result.nodes = {source};
        result.cost = 0.0;
        return true;
    }

    // 检查源点和目标点是否被禁用
    if (isBannedNode(source, bannedNodes) || isBannedNode(target, bannedNodes)) return false;

    // 初始化迪杰斯特拉算法的数据结构
    const int n = graph.vertexCount();
    const double inf = std::numeric_limits<double>::infinity();

    // 距离数组、父节点数组、访问标记
    std::vector<double> dist(n, inf);
    std::vector<int> parent(n, -1);
    std::vector<bool> visited(n, false);

    // 优先队列中的状态结构：包含距离和顶点ID
    struct State {
        double d;
        int v;
    };
    // 比较器：使用最小堆，距离小的节点优先出队
    struct Cmp {
        bool operator()(const State& a, const State& b) const {
            return a.d > b.d; // min-heap by dist
        }
    };

    // 初始化优先队列和源点距离
    std::priority_queue<State, std::vector<State>, Cmp> pq;

    dist[source] = 0.0;
    pq.push({0.0, source});

    // 主循环：从优先队列中取距离最小的节点进行松弛
    while (!pq.empty()) {
        State cur = pq.top();
        pq.pop();

        // 若节点已访问，跳过（防止重复处理）
        int u = cur.v;
        if (visited[u]) continue;
        visited[u] = true;

        // 到达目标点，提前终止
        if (u == target) break;

        // 遍历从u出发的所有边，进行松弛操作
        for (const Edge& e : graph.adjEdges(u)) {
            int v = e.to;

            // 跳过被禁用的节点或边
            if (isBannedNode(v, bannedNodes)) continue;
            if (isBannedEdge(u, v, bannedEdges)) continue;

            // 验证边的权值有效
            double w = e.weight;
            if (!isValidWeight(w)) continue;

            // 松弛边(u,v)：尝试通过u到达v，如果距离更短则更新
            // dist[u]可能是inf，这里保守处理
            double nd = dist[u] + w;
            if (nd < dist[v]) {
                dist[v] = nd;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    if (dist[target] == inf) {
        return false;
    }

    // 从目标点反向回溯源点，还原最短路径
    std::vector<int> rev;
    for (int v = target; v != -1; v = parent[v]) {
        rev.push_back(v);
        if (v == source) break;
    }
    if (rev.empty() || rev.back() != source) {
        return false;
    }

    std::reverse(rev.begin(), rev.end());
    result.nodes = std::move(rev);
    result.cost = dist[target];
    return true;
}
