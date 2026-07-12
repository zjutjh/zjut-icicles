#include "Graph.h"

Graph::Graph() = default;
Graph::~Graph() = default;

// 添加顶点，返回分配的ID
int Graph::addVertex(const std::string& name, double x, double y) {
    int id = static_cast<int>(mVertices.size());
    mVertices.emplace_back(id, name, x, y);
    mAdjList.emplace_back();
    return id;
}

// 删除顶点及其关联的所有边
bool Graph::removeVertex(int id) {
    if (!isValidId(id)) return false;

    // 删除与该点相关的所有边
    // 遍历该顶点的所有邻接边
    for (const Edge& e : mAdjList[id]) {
        int v = e.to;
        // 从对端邻接表中移除
        auto& list = mAdjList[v];
        for (auto it = list.begin(); it != list.end(); ) {
            if (it->to == id) it = list.erase(it);
            else ++it;
        }
        // 从索引中移除双向边
        mEdgeIndex.erase(encodeEdge(id, v));
        mEdgeIndex.erase(encodeEdge(v, id));
    }

    // 清空自身邻接
    mAdjList[id].clear();

    // 标记顶点为无效（保持 id 稳定，便于课设）
    mVertices[id].setName("[removed]");
    return true;
}

// 添加边（无向图，自动双向添加），若边已存在则更新权值
bool Graph::addEdge(int u, int v, double weight) {
    // 验证两个顶点有效且不相同
    if (!isValidId(u) || !isValidId(v) || u == v) return false;

    // 验证边权值有效性（必须为正数且不超过1e18）
    if (weight <= 0.0 || weight > 1e18) return false;

    // 若已存在，更新权值
    if (hasEdge(u, v)) {
        mEdgeIndex.insertOrAssign(encodeEdge(u, v), weight);
        mEdgeIndex.insertOrAssign(encodeEdge(v, u), weight);

        for (Edge& e : mAdjList[u]) if (e.to == v) e.weight = weight;
        for (Edge& e : mAdjList[v]) if (e.to == u) e.weight = weight;
        return true;
    }

    // 新增
    mAdjList[u].emplace_back(u, v, weight);
    mAdjList[v].emplace_back(v, u, weight);

    mEdgeIndex.insert(encodeEdge(u, v), weight);
    mEdgeIndex.insert(encodeEdge(v, u), weight);
    return true;
}

// 删除两顶点间的边（无向图，同时删除两个方向）
bool Graph::removeEdge(int u, int v) {
    if (!hasEdge(u, v)) return false;

    auto& lu = mAdjList[u];
    for (auto it = lu.begin(); it != lu.end(); ) {
        if (it->to == v) it = lu.erase(it);
        else ++it;
    }

    auto& lv = mAdjList[v];
    for (auto it = lv.begin(); it != lv.end(); ) {
        if (it->to == u) it = lv.erase(it);
        else ++it;
    }

    mEdgeIndex.erase(encodeEdge(u, v));
    mEdgeIndex.erase(encodeEdge(v, u));
    return true;
}

// 检查顶点是否存在
bool Graph::hasVertex(int id) const {
    return isValidId(id);
}

// 检查两顶点间是否存在边
bool Graph::hasEdge(int u, int v) const {
    if (!isValidId(u) || !isValidId(v)) return false;
    return mEdgeIndex.contains(encodeEdge(u, v));
}

// 获取边的权值，若边不存在返回无穷大
double Graph::getWeight(int u, int v) const {
    const double* w = mEdgeIndex.find(encodeEdge(u, v));
    if (!w) return std::numeric_limits<double>::infinity();
    return *w;
}

// 获取顶点u的所有邻接边
const std::vector<Edge>& Graph::adjEdges(int u) const {
    static const std::vector<Edge> empty;
    if (!isValidId(u)) return empty;
    return mAdjList[u];
}

// 获取顶点u的所有邻接顶点
std::vector<int> Graph::neighbors(int u) const {
    std::vector<int> result;
    if (!isValidId(u)) return result;
    for (const Edge& e : mAdjList[u]) {
        result.push_back(e.to);
    }
    return result;
}

// 获取图中顶点总数
int Graph::vertexCount() const {
    return static_cast<int>(mVertices.size());
}

// 获取所有顶点
const std::vector<Vertex>& Graph::vertices() const {
    return mVertices;
}

// 清空图中所有顶点和边
void Graph::clear() {
    mVertices.clear();
    mAdjList.clear();
    mEdgeIndex.clear();
}

// 检查顶点ID是否有效
bool Graph::isValidId(int id) const {
    return id >= 0 && id < static_cast<int>(mVertices.size());
}

// 将两个顶点ID编码为唯一的长整数用于索引
long long Graph::encodeEdge(int u, int v) const {
    // 假设顶点数 < 1e6，足够课设使用
    // 利用进制编码：u作为高位，v作为低位
    return static_cast<long long>(u) * 1000000LL + v;
}
