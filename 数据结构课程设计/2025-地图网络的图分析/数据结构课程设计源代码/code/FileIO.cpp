#include "FileIO.h"
#include <fstream>
#include <sstream>
#include <iostream>

// 从文件加载地图数据到图中
bool loadMapFile(const std::string& fileName, Graph& graph) {
    std::ifstream fin(fileName);
    if (!fin.is_open()) {
        std::cerr << "打开文件失败: " << fileName << "\n";
        return false;
    }

    graph.clear();

    std::string line;

    // 跳过注释，读取 vertexCount edgeCount
    int vertexCount = 0;
    int edgeCount = 0;
    while (std::getline(fin, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        ss >> vertexCount >> edgeCount;
        break;
    }

    if (vertexCount <= 0) {
        std::cerr << "顶点数量无效\n";
        return false;
    }

    // 读取顶点
    int readVertices = 0;
    while (readVertices < vertexCount && std::getline(fin, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::stringstream ss(line);
        int id;
        std::string name;
        double x, y;
        if (!(ss >> id >> name >> x >> y)) {
            std::cerr << "顶点数据格式错误\n";
            return false;
        }

        // 验证顶点名称不为空
        if (name.empty()) {
            std::cerr << "顶点名称不能为空\n";
            return false;
        }

        // 验证顶点ID是否与预期顺序匹配（应为 0,1,2,...）
        if (id != readVertices) {
            std::cerr << "顶点ID顺序错误：期望 " << readVertices << " 但得到 " << id << "\n";
            return false;
        }

        // 验证坐标值的有效性（合理范围：-1e6 到 1e6）
        if (x < -1e6 || x > 1e6 || y < -1e6 || y > 1e6) {
            std::cerr << "顶点坐标超出有效范围: (" << x << ", " << y << ")\n";
            return false;
        }

        // 按顺序 addVertex，id 用于校验
        int newId = graph.addVertex(name, x, y);
        if (newId != id) {
            // addVertex 返回 -1 表示名称验证失败
            std::cerr << "添加顶点失败: " << name << " （可能名称无效或过长）\n";
            return false;
        }
        readVertices++;
    }

    // 验证读取的顶点数量与声明的数量匹配
    if (readVertices != vertexCount) {
        std::cerr << "顶点数量不匹配：期望 " << vertexCount << " 但读取 " << readVertices << "\n";
        return false;
    }

    // 读取边
    int readEdges = 0;
    while (readEdges < edgeCount && std::getline(fin, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::stringstream ss(line);
        int u, v;
        double w;
        if (!(ss >> u >> v >> w)) {
            std::cerr << "边数据格式错误\n";
            return false;
        }

        // 验证边权值的有效性（必须为正数且不超过1e18）
        if (w <= 0.0 || w > 1e18) {
            std::cerr << "边权值无效: " << w << "（必须在 (0, 1e18] 范围内）\n";
            return false;
        }

        if (!graph.addEdge(u, v, w)) {
            std::cerr << "添加边失败: " << u << " " << v << "\n";
            return false;
        }
        readEdges++;
    }

    // 验证读取的边数量与声明的数量匹配
    if (readEdges != edgeCount) {
        std::cerr << "边数量不匹配：期望 " << edgeCount << " 但读取 " << readEdges << "\n";
        return false;
    }

    fin.close();
    return true;
}

// 将图的数据保存到文件中
bool saveMapFile(const std::string& fileName, const Graph& graph) {
    std::ofstream fout(fileName);
    if (!fout.is_open()) {
        std::cerr << "写入文件失败: " << fileName << "\n";
        return false;
    }

    const auto& vertices = graph.vertices();
    int vertexCount = static_cast<int>(vertices.size());

    // 统计边数（因为是无向图，每条边只记录一次）
    int edgeCount = 0;
    for (int u = 0; u < vertexCount; u++) {
        for (const Edge& e : graph.adjEdges(u)) {
            if (e.from < e.to) edgeCount++;
        }
    }

    fout << "# 图导航系统的地图文件\n";
    fout << vertexCount << " " << edgeCount << "\n\n";

    fout << "# 顶点: id 名称 x坐标 y坐标\n";
    for (const Vertex& v : vertices) {
        fout << v.id() << " "
             << v.name() << " "
             << v.x() << " "
             << v.y() << "\n";
    }

    fout << "\n# 边: 起点 终点 权重\n";
    for (int u = 0; u < vertexCount; u++) {
        for (const Edge& e : graph.adjEdges(u)) {
            if (e.from < e.to) {
                fout << e.from << " "
                     << e.to << " "
                     << e.weight << "\n";
            }
        }
    }

    fout.close();
    return true;
}
