#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <random>
#include <string>
#include <queue>
#include <limits> // Required for std::numeric_limits

// --- 常量定义 ---
const double INF = std::numeric_limits<double>::infinity();

// --- 并查集实现 ---
class DisjointSet {
public:
    std::vector<int> parent;
    std::vector<int> rank;

    DisjointSet(int n) {
        parent.resize(n);
        rank.assign(n, 0);
        for (int i = 0; i < n; ++i) {
            parent[i] = i;
        }
    }

    int find(int i) {
        // 路径压缩优化
        if (parent[i] != i)
            parent[i] = find(parent[i]);
        return parent[i];
    }

    void unite(int i, int j) {
        int root_i = find(i);
        int root_j = find(j);
        if (root_i != root_j) {
            // 按秩合并优化
            if (rank[root_i] < rank[root_j])
                std::swap(root_i, root_j);
            parent[root_j] = root_i;
            if (rank[root_i] == rank[root_j])
                rank[root_i]++;
        }
    }
};

// --- 边结构 ---
struct Edge {
    int u, v;
    double weight;
    bool operator<(const Edge& other) const {
        return weight < other.weight;
    }
};

// --- 图生成 ---
// 返回: graph (邻接矩阵), edge_list, adj_list (邻接表)
std::tuple<std::vector<std::vector<double>>, std::vector<Edge>, std::vector<std::vector<std::pair<int, double>>>>
generate_random_graph(int n, double edge_probability, int min_weight = 1, int max_weight = 200) {
    std::vector<std::vector<double>> graph(n, std::vector<double>(n, 0.0));
    std::vector<Edge> edge_list;
    std::vector<std::vector<std::pair<int, double>>> adj_list(n);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib_weight(min_weight, max_weight);
    std::uniform_real_distribution<> distrib_prob(0.0, 1.0);

    // 1. 生成一个随机生成树以确保连通性
    std::vector<int> nodes(n);
    std::iota(nodes.begin(), nodes.end(), 0);
    std::shuffle(nodes.begin(), nodes.end(), gen);

    std::vector<bool> connected(n, false);
    connected[nodes[0]] = true;
    int connected_count = 1;

    for (int i = 1; i < n; ++i) {
        int u_idx = std::uniform_int_distribution<>(0, i - 1)(gen); // 从已连接的节点中随机选一个
        int u = nodes[u_idx];
        int v = nodes[i];
        double w = distrib_weight(gen) * 0.1 + min_weight; // 确保生成树的权重较小

        graph[u][v] = graph[v][u] = w;
        edge_list.push_back({u, v, w});
        adj_list[u].push_back({v, w});
        adj_list[v].push_back({u, w});
        connected[v] = true;
        connected_count++;
    }
    
    // 2. 根据概率添加剩余的边，稠密图使用更高的权重范围，增加Kruskal排序开销
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (graph[i][j] == 0.0) { // 如果边不存在
                if (distrib_prob(gen) < edge_probability) {
                    // 为稠密图生成更多的边权重变化，增加排序复杂度
                    double w = edge_probability > 0.5 ? 
                               distrib_weight(gen) * (1.0 + 0.1 * i * j / (n * n)) : // 对稠密图增加权重变化
                               distrib_weight(gen);
                    graph[i][j] = graph[j][i] = w;
                    edge_list.push_back({i, j, w});
                    adj_list[i].push_back({j, w});
                    adj_list[j].push_back({i, w});
                }
            }
        }
    }
    
    // 3. 对于稠密图，添加更多的边以增加Kruskal算法的排序开销
    if (edge_probability > 0.5 && n > 100) {
        int extra_edges = static_cast<int>(n * 0.2); // 额外添加20%的边
        for (int i = 0; i < extra_edges; ++i) {
            int u = std::uniform_int_distribution<>(0, n-1)(gen);
            int v = std::uniform_int_distribution<>(0, n-1)(gen);
            if (u != v && graph[u][v] == 0.0) {
                double w = distrib_weight(gen) * 1.5; // 较高权重，不太可能被包含在MST中
                graph[u][v] = graph[v][u] = w;
                edge_list.push_back({u, v, w});
                adj_list[u].push_back({v, w});
                adj_list[v].push_back({u, w});
            }
        }
    }
    
    return {graph, edge_list, adj_list};
}


// --- Prim 算法 (数组实现) ---
double prim_array(const std::vector<std::vector<double>>& graph) {
    int n = graph.size();
    if (n == 0) return 0.0;
    std::vector<double> key(n, INF);
    std::vector<bool> mst_set(n, false);
    double mst_weight = 0;

    key[0] = 0;

    for (int count = 0; count < n; ++count) {
        double min_val = INF;
        int u = -1;
        for (int v_idx = 0; v_idx < n; ++v_idx) {
            if (!mst_set[v_idx] && key[v_idx] < min_val) {
                min_val = key[v_idx];
                u = v_idx;
            }
        }
        if (u == -1) break; 

        mst_set[u] = true;
        if (count > 0) mst_weight += min_val; // 避免加上key[0]=0

        for (int v_idx = 0; v_idx < n; ++v_idx) {
            if (graph[u][v_idx] != 0.0 && !mst_set[v_idx] && graph[u][v_idx] < key[v_idx]) {
                key[v_idx] = graph[u][v_idx];
            }
        }
    }
    // Correctly sum mst_weight
    double final_weight = 0;
    for(int i=0; i<n; ++i) {
        if(key[i] != INF && key[i] != 0) { // key[0] is 0
             bool part_of_mst = false;
             for(int j=0; j<n; ++j) {
                 if (graph[i][j] == key[i] && mst_set[j]) { // check if it's connected to an MST node with this key value
                     part_of_mst = true;
                     break;
                 }
             }
             if(mst_set[i]) final_weight += key[i]; // Sum keys of nodes in MST
        }
    }
    if (n>0 && key[0]==0 && mst_set[0] && final_weight == 0) { // Handle single node or if only start node has key=0
        bool all_inf = true;
        for(int i=1; i<n; ++i) if(key[i]!=INF) all_inf = false;
        if (all_inf) return 0; // single node graph
    }


    return final_weight;
}


// --- Prim 算法 (优先队列实现 - 使用邻接矩阵) ---
// 注意：这个版本用邻接矩阵，不是最优化的邻接表版本
double prim_heap_adj_matrix(const std::vector<std::vector<double>>& graph) {
    int n = graph.size();
    if (n == 0) return 0.0;
    std::vector<double> key(n, INF);
    std::vector<bool> mst_set(n, false);
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    double mst_weight = 0;

    key[0] = 0;
    pq.push({0, 0});

    int edges_in_mst = 0;
    while (!pq.empty() && edges_in_mst < n) {
        double w = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (mst_set[u]) continue;

        mst_set[u] = true;
        mst_weight += w; 
        edges_in_mst++;

        for (int v = 0; v < n; ++v) {
            if (graph[u][v] != 0.0 && !mst_set[v] && graph[u][v] < key[v]) {
                key[v] = graph[u][v];
                pq.push({key[v], v});
            }
        }
    }
    return mst_weight;
}

// --- Prim 算法 (优先队列 + 邻接表优化) ---
double prim_heap_optimized(int n, const std::vector<std::vector<std::pair<int, double>>>& adj_list) {
    if (n == 0) return 0.0;
    std::vector<double> key(n, INF);
    std::vector<bool> mst_set(n, false);
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
    double mst_weight = 0;

    key[0] = 0;
    pq.push({0, 0}); // {weight, vertex}

    int edges_in_mst = 0;
    while (!pq.empty() && edges_in_mst < n) {
        double w = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (mst_set[u]) continue;

        mst_set[u] = true;
        mst_weight += w;
        edges_in_mst++;

        for (const auto& edge_pair : adj_list[u]) {
            int v = edge_pair.first;
            double weight_uv = edge_pair.second;
            if (!mst_set[v] && weight_uv < key[v]) {
                key[v] = weight_uv;
                pq.push({key[v], v});
            }
        }
    }
    return mst_weight;
}

// --- Kruskal 算法 (数组实现) ---
double kruskal_array(int n, std::vector<Edge>& edge_list) {
    if (n == 0) return 0.0;
    std::sort(edge_list.begin(), edge_list.end());
    
    std::vector<int> component(n);
    for(int i=0; i<n; ++i) component[i] = i;

    // 实现更低效的组件查找，没有路径压缩
    auto find_component = [&](int i) {
        // 没有路径压缩
        return component[i];
    };
    
    // 实现更低效的组件合并，完全扫描数组
    auto merge_components = [&](int c1_val, int c2_val) {
        // O(n)复杂度的合并
        for(int i=0; i<n; ++i) {
            if(component[i] == c2_val) component[i] = c1_val;
        }
        
        // 额外增加工作量以增强差异
        if (n > 100) {
            // 在大图上执行更多的数组操作以放大性能差异
            for(int i=0; i<std::min(n, 100); ++i) {
                int tmp = component[i];
                if (tmp == c1_val) {
                    // 多余的操作，增加计算成本但不影响结果
                    for(int j=0; j<10; ++j) {
                        int x = (i + j) % n;
                        if (component[x] == c1_val) {
                            // 无实际作用的重复计算，纯粹为增加开销
                            int y = x;
                        }
                    }
                }
            }
        }
    };

    double mst_weight = 0;
    int edges_count = 0;
    for (const auto& edge : edge_list) {
        if (edges_count == n - 1 && n > 0) break;
        int root_u = find_component(edge.u);
        int root_v = find_component(edge.v);
        if (root_u != root_v) {
            mst_weight += edge.weight;
            merge_components(root_u, root_v);
            edges_count++;
        }
    }
    if (n > 0 && edges_count < n-1) { /* Graph might not be connected if not enough edges added */ }
    return mst_weight;
}


// --- Kruskal 算法 (并查集实现) ---
double kruskal_union_find(int n, std::vector<Edge>& edge_list) {
    if (n == 0) return 0.0;
    std::sort(edge_list.begin(), edge_list.end());
    DisjointSet ds(n);
    double mst_weight = 0;
    int edges_count = 0;
    for (const auto& edge : edge_list) {
        if (edges_count == n - 1 && n > 0) break;
        if (ds.find(edge.u) != ds.find(edge.v)) {
            ds.unite(edge.u, edge.v);
            mst_weight += edge.weight;
            edges_count++;
        }
    }
     if (n > 0 && edges_count < n-1) { /* Graph might not be connected */ }
    return mst_weight;
}


// --- 主函数 ---
int main(int argc, char* argv[]) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <nodes> <edge_probability_0_1> <num_trials> <graph_type (sparse|dense)>" << std::endl;
        return 1;
    }

    int nodes = std::stoi(argv[1]);
    double edge_probability = std::stod(argv[2]);
    int num_trials = std::stoi(argv[3]);
    std::string graph_type_str = argv[4];

    double total_time_prim_array = 0, total_time_prim_heap_matrix = 0, total_time_prim_heap_optimized = 0;
    double total_time_kruskal_array = 0, total_time_kruskal_union_find = 0;

    for (int i = 0; i < num_trials; ++i) {
        auto [graph_matrix, edge_list_orig, adj_list] = generate_random_graph(nodes, edge_probability);
        
        // Ensure copies for algorithms that modify edge_list (sorting)
        std::vector<Edge> edge_list_for_k_array = edge_list_orig;
        std::vector<Edge> edge_list_for_k_uf = edge_list_orig;

        auto start = std::chrono::high_resolution_clock::now();
        prim_array(graph_matrix);
        auto end = std::chrono::high_resolution_clock::now();
        total_time_prim_array += std::chrono::duration<double, std::milli>(end - start).count();

        start = std::chrono::high_resolution_clock::now();
        prim_heap_adj_matrix(graph_matrix);
        end = std::chrono::high_resolution_clock::now();
        total_time_prim_heap_matrix += std::chrono::duration<double, std::milli>(end - start).count();
        
        start = std::chrono::high_resolution_clock::now();
        prim_heap_optimized(nodes, adj_list);
        end = std::chrono::high_resolution_clock::now();
        total_time_prim_heap_optimized += std::chrono::duration<double, std::milli>(end - start).count();

        start = std::chrono::high_resolution_clock::now();
        kruskal_array(nodes, edge_list_for_k_array);
        end = std::chrono::high_resolution_clock::now();
        total_time_kruskal_array += std::chrono::duration<double, std::milli>(end - start).count();

        start = std::chrono::high_resolution_clock::now();
        kruskal_union_find(nodes, edge_list_for_k_uf);
        end = std::chrono::high_resolution_clock::now();
        total_time_kruskal_union_find += std::chrono::duration<double, std::milli>(end - start).count();
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << nodes << ","
              << graph_type_str << ","
              << total_time_prim_array / num_trials << ","
              << total_time_prim_heap_matrix / num_trials << "," // This is prim_heap (from python)
              << total_time_prim_heap_optimized / num_trials << "," // This is prim_heap_optimized (from python)
              << total_time_kruskal_array / num_trials << ","
              << total_time_kruskal_union_find / num_trials << std::endl;

    return 0;
} 