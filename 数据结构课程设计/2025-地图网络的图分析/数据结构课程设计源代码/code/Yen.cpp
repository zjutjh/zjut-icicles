#include "Yen.h"
#include "Dijkstra.h"
#include <queue>
#include <string>
#include <vector>
#include <limits>

namespace {
    // 编码边的键（与Dijkstra中的编码方式相同）
    long long encodeEdgeKey(int u, int v) {
        return static_cast<long long>(u) * 1000000LL + v;
    }

    // 计算路径前缀 [0..endIndex] 的代价（endIndex >= 0）
    // 例如 nodes = [s,a,b,t]，endIndex=2 => s->a->b 的代价
    double prefixCost(const Graph& graph, const std::vector<int>& nodes, int endIndex) {
        if (endIndex <= 0) return 0.0;
        double sum = 0.0;
        for (int i = 0; i < endIndex; i++) {
            int u = nodes[i];
            int v = nodes[i + 1];
            double w = graph.getWeight(u, v);
            if (w == std::numeric_limits<double>::infinity()) {
                return std::numeric_limits<double>::infinity();
            }
            sum += w;
        }
        return sum;
    }

    std::string pathSignature(const std::vector<int>& nodes) {
        // 简单签名：用 '-' 连接，避免候选路径重复
        // 例如 0-3-5-2
        std::string s;
        s.reserve(nodes.size() * 4);
        for (std::size_t i = 0; i < nodes.size(); i++) {
            s += std::to_string(nodes[i]);
            if (i + 1 < nodes.size()) s += "-";
        }
        return s;
    }

    struct Candidate {
        Path path;
        std::string sig;
    };

    struct CandidateCmp {
        bool operator()(const Candidate& a, const Candidate& b) const {
            return a.path.cost > b.path.cost; // min-heap
        }
    };
}

// Yen算法：求取第K条最短路径（K=2时求次短路）
// 参数：图，源点，目标点，返回最短路径，返回次短路径
// 返回值：是否找到至少一条有效路径
bool yenK2(
    const Graph& graph,
    int source,
    int target,
    Path& shortestPath,
    Path& secondShortestPath
) {
    // 初始化返回的路径
    shortestPath.nodes.clear();
    shortestPath.cost = 0.0;
    secondShortestPath.nodes.clear();
    secondShortestPath.cost = 0.0;

    // 第0步：先求最短路 P1
    if (!dijkstraShortestPath(graph, source, target, nullptr, nullptr, shortestPath)) {
        return false;
    }

    // 如果最短路只有一个点（source==target），次短路一般无意义
    if (shortestPath.nodes.size() <= 1) {
        return true;
    }

    // 候选集合（按代价最小出堆）
    std::priority_queue<Candidate, std::vector<Candidate>, CandidateCmp> candidates;

    // 用 RBTree 做去重（你也可以改成别的，但这里保持“全 RBTree”风格）
    RBTree<std::string, bool> seenCandidate;

    const std::vector<int>& p1 = shortestPath.nodes;
    const int len = static_cast<int>(p1.size());

    // 第1步：枚举分支点i（终点不作为分支点）
    for (int i = 0; i <= len - 2; i++) {
        int spurNode = p1[i];

        // 根路径 = p1[0..i]
        // 禁用节点：根路径中除分支点外的节点都禁用，保证简单路径
        BannedNodeSet bannedNodes;
        for (int k = 0; k <= i - 1; k++) {
            bannedNodes.insertOrAssign(p1[k], true);
        }

        // 禁用边：禁用让分支点继续沿着P1走的那条边，从而强制分叉
        BannedEdgeSet bannedEdges;
        int nextNodeOnP1 = p1[i + 1];
        bannedEdges.insertOrAssign(encodeEdgeKey(spurNode, nextNodeOnP1), true);
        // 无向图：反向也禁用更安全
        bannedEdges.insertOrAssign(encodeEdgeKey(nextNodeOnP1, spurNode), true);

        // 分支路径：从分支点到目标点
        Path spurPath;
        bool ok = dijkstraShortestPath(graph, spurNode, target, &bannedNodes, &bannedEdges, spurPath);
        if (!ok) continue;

        // 拼接总路径 = 根路径[:-1] + 分支路径
        std::vector<int> totalNodes;
        totalNodes.reserve((i + 1) + spurPath.nodes.size());

        // 根路径去掉最后一个分支点，避免重复
        for (int k = 0; k < i; k++) {
            totalNodes.push_back(p1[k]);
        }
        // 加上分支路径（第一个就是分支点）
        for (int v : spurPath.nodes) {
            totalNodes.push_back(v);
        }

        // 代价 = 根路径代价(0..i) + 分支路径代价
        // 注意：根路径代价包含到分支点的代价，所以 endIndex=i
        double rootCost = prefixCost(graph, p1, i);
        if (rootCost == std::numeric_limits<double>::infinity()) continue;

        Path totalPath;
        totalPath.nodes = std::move(totalNodes);
        totalPath.cost = rootCost + spurPath.cost;

        // 去重：相同节点序列只保留一次
        std::string sig = pathSignature(totalPath.nodes);
        if (seenCandidate.contains(sig)) continue;
        seenCandidate.insertOrAssign(sig, true);

        candidates.push({totalPath, sig});
    }

    // 第2步：取最小候选作为第2条最短路
    if (candidates.empty()) {
        // 次短路不存在
        return true;
    }

    secondShortestPath = candidates.top().path;
    return true;
}
