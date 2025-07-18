import subprocess
import matplotlib.pyplot as plt
import csv
import os
import numpy as np # For np.log

# --- 配置 ---
CPP_SOURCE_FILE = "mst_experiment.cpp"
CPP_EXECUTABLE_FILE = "mst_experiment.exe" # 或者在Linux/macOS上是 "mst_experiment"

NODE_COUNTS = [50, 100, 200, 300, 400, 600, 800] # 使用更大规模的图，增加差异
EDGE_PROBABILITIES = {
    "sparse": 0.005,  # 更改为更稀疏
    "dense": 0.9    # 更改为更稠密
}
NUM_TRIALS = 5 # 每个配置的试验次数
OUTPUT_DIR = "img_cpp"
DATA_DIR = os.path.join(OUTPUT_DIR, "data_cpp")

# --- 编译 C++ 代码 (如果需要) ---
def compile_cpp():
    # 检查可执行文件是否存在并且比源文件新
    if os.path.exists(CPP_EXECUTABLE_FILE) and \
       os.path.getmtime(CPP_EXECUTABLE_FILE) > os.path.getmtime(CPP_SOURCE_FILE):
        print(f"{CPP_EXECUTABLE_FILE} 是最新的，跳过编译。")
        return True

    print(f"正在编译 {CPP_SOURCE_FILE} 到 {CPP_EXECUTABLE_FILE}...")
    compile_command = ["g++", "-std=c++17", "-O2", CPP_SOURCE_FILE, "-o", CPP_EXECUTABLE_FILE]
    try:
        process = subprocess.run(compile_command, check=True, capture_output=True, text=True)
        print("C++ 代码编译成功。")
        if process.stdout:
            print("Compiler output:", process.stdout)
        return True
    except subprocess.CalledProcessError as e:
        print(f"C++ 代码编译失败。错误代码: {e.returncode}")
        print("错误信息:", e.stderr)
        return False
    except FileNotFoundError:
        print("错误: g++ 编译器未找到。请确保已安装并配置在系统路径中。")
        return False

# --- 运行 C++ 实验 ---
def run_cpp_experiment(nodes, probability, num_trials, graph_type):
    command = [f".\\{CPP_EXECUTABLE_FILE}", str(nodes), str(probability), str(num_trials), graph_type]
    try:
        process = subprocess.run(command, check=True, capture_output=True, text=True)
        # 输出格式: nodes,type,prim_array,prim_heap,prim_optimized,kruskal_array,kruskal_union_find
        parts = process.stdout.strip().split(',')
        if len(parts) == 7:
            return {
                "nodes": int(parts[0]),
                "type": parts[1],
                "prim_array": float(parts[2]),
                "prim_heap": float(parts[3]), # Corresponds to prim_heap_adj_matrix in C++
                "prim_heap_optimized": float(parts[4]),
                "kruskal_array": float(parts[5]),
                "kruskal_union_find": float(parts[6])
            }
        else:
            print(f"错误：C++输出格式不正确: {process.stdout.strip()}")
            return None
    except subprocess.CalledProcessError as e:
        print(f"运行C++实验失败 (节点数: {nodes}, 类型: {graph_type})。错误代码: {e.returncode}")
        print("错误信息:", e.stderr)
        return None
    except FileNotFoundError:
        print(f"错误: 未找到C++可执行文件 '{CPP_EXECUTABLE_FILE}'。请先编译。")
        return None

# --- 主实验逻辑 ---
def run_all_experiments():
    if not compile_cpp():
        return None

    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
    if not os.path.exists(DATA_DIR):
        os.makedirs(DATA_DIR)

    all_results_data = [] # 存储原始C++输出

    # 重新组织 results 结构以匹配之前的 Python 脚本
    results = {
        'prim_array': {'sparse': [], 'dense': []},
        'prim_heap': {'sparse': [], 'dense': []}, # prim_heap_adj_matrix from C++
        'prim_heap_optimized': {'sparse': [], 'dense': []},
        'kruskal_array': {'sparse': [], 'dense': []},
        'kruskal_union_find': {'sparse': [], 'dense': []}
    }
    
    # 临时列表来收集每个节点数的平均时间
    # 这些列表将用于填充上面的 `results` 字典
    temp_times = {algo: {g_type: [] for g_type in EDGE_PROBABILITIES.keys()} for algo in results.keys()}

    print("开始运行C++实验...")
    for n_nodes in NODE_COUNTS:
        print(f"处理节点数: {n_nodes}")
        # 为当前节点数重置时间收集器
        current_node_times = {algo: {g_type: [] for g_type in EDGE_PROBABILITIES.keys()} for algo in results.keys()}

        for graph_type, prob in EDGE_PROBABILITIES.items():
            print(f"  类型: {graph_type} (概率: {prob})")
            # C++程序内部处理 num_trials 并返回平均值
            # 因此这里我们只需要调用一次 C++ 程序获取该配置的平均结果
            exp_result = run_cpp_experiment(n_nodes, prob, NUM_TRIALS, graph_type)
            if exp_result:
                all_results_data.append(exp_result)
                # 填充 results 字典
                results['prim_array'][graph_type].append(exp_result['prim_array'])
                results['prim_heap'][graph_type].append(exp_result['prim_heap'])
                results['prim_heap_optimized'][graph_type].append(exp_result['prim_heap_optimized'])
                results['kruskal_array'][graph_type].append(exp_result['kruskal_array'])
                results['kruskal_union_find'][graph_type].append(exp_result['kruskal_union_find'])
            else:
                # 如果实验失败，填充NaN或0以保持数组长度一致
                for algo in results.keys():
                    results[algo][graph_type].append(float('nan')) 

    # 保存原始C++数据
    if all_results_data:
        with open(os.path.join(DATA_DIR, "cpp_raw_results.csv"), "w", newline="") as f:
            if all_results_data:
                writer = csv.DictWriter(f, fieldnames=all_results_data[0].keys())
                writer.writeheader()
                writer.writerows(all_results_data)
        print(f"原始C++实验数据已保存到 {DATA_DIR}/cpp_raw_results.csv")
    
    return results

# --- 绘图函数 (与之前Python脚本中的plot_results类似) ---
def plot_all_results(node_counts_list, results_dict, output_dir_path):
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams.update({'font.size': 10})

    # 图1: Prim算法不同实现比较 (稀疏图)
    plt.figure(figsize=(10, 6))
    prim_nodes = node_counts_list # 使用所有节点数
    if results_dict['prim_array']['sparse'] and results_dict['prim_heap']['sparse']:
        plt.plot(prim_nodes, results_dict['prim_array']['sparse'][:len(prim_nodes)], marker='o', linewidth=2, markersize=8, label='Prim (数组 - 邻接矩阵)')
        plt.plot(prim_nodes, results_dict['prim_heap']['sparse'][:len(prim_nodes)], marker='s', linewidth=2, markersize=8, label='Prim (优先队列 - 邻接矩阵)')
        plt.plot(prim_nodes, results_dict['prim_heap_optimized']['sparse'][:len(prim_nodes)], marker='^', linewidth=2, markersize=8, label='Prim (优先队列 - 邻接表)')
        
        # 添加数据标签
        for i, node in enumerate(prim_nodes):
            plt.annotate(f'{results_dict["prim_array"]["sparse"][i]:.2f}', 
                        xy=(node, results_dict["prim_array"]["sparse"][i]),
                        xytext=(0, 10), textcoords='offset points',
                        ha='center', fontsize=8, bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))
            plt.annotate(f'{results_dict["prim_heap_optimized"]["sparse"][i]:.2f}', 
                        xy=(node, results_dict["prim_heap_optimized"]["sparse"][i]),
                        xytext=(0, -15), textcoords='offset points',
                        ha='center', fontsize=8, bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))
    plt.xlabel('节点数量', fontsize=12)
    plt.ylabel('平均运行时间 (毫秒)', fontsize=12)
    plt.title('Prim算法不同实现在稀疏图上的性能 (C++)', fontsize=14)
    plt.legend(fontsize=10)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir_path, "prim_sparse_comparison_cpp.png"), dpi=300, bbox_inches='tight')
    plt.close()

    # 图2: Prim算法不同实现比较 (稠密图)
    plt.figure(figsize=(10, 6))
    if results_dict['prim_array']['dense'] and results_dict['prim_heap']['dense']:
        plt.plot(prim_nodes, results_dict['prim_array']['dense'][:len(prim_nodes)], marker='o', linewidth=2, markersize=8, label='Prim (数组 - 邻接矩阵)')
        plt.plot(prim_nodes, results_dict['prim_heap']['dense'][:len(prim_nodes)], marker='s', linewidth=2, markersize=8, label='Prim (优先队列 - 邻接矩阵)')
        plt.plot(prim_nodes, results_dict['prim_heap_optimized']['dense'][:len(prim_nodes)], marker='^', linewidth=2, markersize=8, label='Prim (优先队列 - 邻接表)')
        
        # 添加数据标签
        for i, node in enumerate(prim_nodes):
            plt.annotate(f'{results_dict["prim_array"]["dense"][i]:.2f}', 
                        xy=(node, results_dict["prim_array"]["dense"][i]),
                        xytext=(0, 10), textcoords='offset points',
                        ha='center', fontsize=8, bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))
            plt.annotate(f'{results_dict["prim_heap_optimized"]["dense"][i]:.2f}', 
                        xy=(node, results_dict["prim_heap_optimized"]["dense"][i]),
                        xytext=(0, -15), textcoords='offset points',
                        ha='center', fontsize=8, bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))
    plt.xlabel('节点数量', fontsize=12)
    plt.ylabel('平均运行时间 (毫秒)', fontsize=12)
    plt.title('Prim算法不同实现在稠密图上的性能 (C++)', fontsize=14)
    plt.legend(fontsize=10)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir_path, "prim_dense_comparison_cpp.png"), dpi=300, bbox_inches='tight')
    plt.close()

    # 图3: Kruskal算法不同实现比较 (稀疏图)
    plt.figure(figsize=(10, 6))
    if results_dict['kruskal_array']['sparse'] and results_dict['kruskal_union_find']['sparse']:
        plt.plot(node_counts_list, results_dict['kruskal_array']['sparse'], marker='o', linewidth=2, markersize=8, label='Kruskal (数组)')
        plt.plot(node_counts_list, results_dict['kruskal_union_find']['sparse'], marker='s', linewidth=2, markersize=8, label='Kruskal (并查集)')
        
        # 添加数据标签和比例标注
        for i, node in enumerate(node_counts_list):
            ratio = results_dict['kruskal_array']['sparse'][i] / results_dict['kruskal_union_find']['sparse'][i] if results_dict['kruskal_union_find']['sparse'][i] > 0 else 0
            plt.annotate(f'{results_dict["kruskal_array"]["sparse"][i]:.2f}', 
                        xy=(node, results_dict["kruskal_array"]["sparse"][i]),
                        xytext=(0, 10), textcoords='offset points',
                        ha='center', fontsize=8, bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))
            plt.annotate(f'{results_dict["kruskal_union_find"]["sparse"][i]:.2f}', 
                        xy=(node, results_dict["kruskal_union_find"]["sparse"][i]),
                        xytext=(0, -15), textcoords='offset points',
                        ha='center', fontsize=8, bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))
            
            if ratio > 1.1:  # 只显示明显的比例
                plt.annotate(f'数组/并查集={ratio:.2f}x', 
                            xy=(node, (results_dict["kruskal_array"]["sparse"][i] + results_dict["kruskal_union_find"]["sparse"][i])/2),
                            xytext=(10, 0), textcoords='offset points',
                            fontsize=9, ha='left', bbox=dict(boxstyle="round,pad=0.3", fc="yellow", alpha=0.4))
    plt.xlabel('节点数量', fontsize=12)
    plt.ylabel('平均运行时间 (毫秒)', fontsize=12)
    plt.title('Kruskal算法不同实现在稀疏图上的性能 (C++)', fontsize=14)
    plt.legend(fontsize=10)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir_path, "kruskal_sparse_comparison_cpp.png"), dpi=300, bbox_inches='tight')
    plt.close()

    # 图4: Kruskal算法不同实现比较 (稠密图)
    plt.figure(figsize=(10, 6))
    if results_dict['kruskal_array']['dense'] and results_dict['kruskal_union_find']['dense']:
        plt.plot(node_counts_list, results_dict['kruskal_array']['dense'], marker='o', linewidth=2, markersize=8, label='Kruskal (数组)')
        plt.plot(node_counts_list, results_dict['kruskal_union_find']['dense'], marker='s', linewidth=2, markersize=8, label='Kruskal (并查集)')
        
        # 添加数据标签和比例标注
        for i, node in enumerate(node_counts_list):
            ratio = results_dict['kruskal_array']['dense'][i] / results_dict['kruskal_union_find']['dense'][i] if results_dict['kruskal_union_find']['dense'][i] > 0 else 0
            plt.annotate(f'{results_dict["kruskal_array"]["dense"][i]:.2f}', 
                        xy=(node, results_dict["kruskal_array"]["dense"][i]),
                        xytext=(0, 10), textcoords='offset points',
                        ha='center', fontsize=8, bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))
            plt.annotate(f'{results_dict["kruskal_union_find"]["dense"][i]:.2f}', 
                        xy=(node, results_dict["kruskal_union_find"]["dense"][i]),
                        xytext=(0, -15), textcoords='offset points',
                        ha='center', fontsize=8, bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))
            
            if ratio > 1.1:  # 只显示明显的比例
                plt.annotate(f'数组/并查集={ratio:.2f}x', 
                            xy=(node, (results_dict["kruskal_array"]["dense"][i] + results_dict["kruskal_union_find"]["dense"][i])/2),
                            xytext=(10, 0), textcoords='offset points',
                            fontsize=9, ha='left', bbox=dict(boxstyle="round,pad=0.3", fc="yellow", alpha=0.4))
    plt.xlabel('节点数量', fontsize=12)
    plt.ylabel('平均运行时间 (毫秒)', fontsize=12)
    plt.title('Kruskal算法不同实现在稠密图上的性能 (C++)', fontsize=14)
    plt.legend(fontsize=10)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir_path, "kruskal_dense_comparison_cpp.png"), dpi=300, bbox_inches='tight')
    plt.close()

    # 图5: 最优Prim vs 最优Kruskal (稀疏图)
    plt.figure(figsize=(10, 6))
    if results_dict['prim_heap_optimized']['sparse'] and results_dict['kruskal_union_find']['sparse']:
        plt.plot(node_counts_list, results_dict['prim_heap_optimized']['sparse'], marker='^', linewidth=2, markersize=8, label='Prim (优先队列 - 邻接表)')
        plt.plot(node_counts_list, results_dict['kruskal_union_find']['sparse'], marker='s', linewidth=2, markersize=8, label='Kruskal (并查集)')
        
        # 添加性能比例标注
        for i, node in enumerate(node_counts_list):
            if results_dict['prim_heap_optimized']['sparse'][i] > 1e-9 and results_dict['kruskal_union_find']['sparse'][i] > 1e-9:
                ratio = results_dict['prim_heap_optimized']['sparse'][i] / results_dict['kruskal_union_find']['sparse'][i]
                plt.annotate(f'P/K={ratio:.2f}x', 
                            xy=(node, (results_dict['prim_heap_optimized']['sparse'][i] + results_dict['kruskal_union_find']['sparse'][i])/2),
                            xytext=(10, 0), textcoords='offset points',
                            fontsize=9, ha='left', bbox=dict(boxstyle="round,pad=0.3", fc="yellow", alpha=0.4))
    plt.xlabel('节点数量', fontsize=12)
    plt.ylabel('平均运行时间 (毫秒)', fontsize=12)
    plt.title('最优Prim与Kruskal在稀疏图上的性能 (C++)', fontsize=14)
    plt.legend(fontsize=10)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir_path, "optimal_sparse_comparison_cpp.png"), dpi=300, bbox_inches='tight')
    plt.close()

    # 图6: 最优Prim vs 最优Kruskal (稠密图)
    plt.figure(figsize=(10, 6))
    if results_dict['prim_heap_optimized']['dense'] and results_dict['kruskal_union_find']['dense']:
        prim_dense_times = results_dict['prim_heap_optimized']['dense']
        kruskal_dense_times = results_dict['kruskal_union_find']['dense']
        plt.plot(node_counts_list, prim_dense_times, marker='^', linewidth=2, markersize=8, label='Prim (优先队列 - 邻接表)')
        plt.plot(node_counts_list, kruskal_dense_times, marker='s', linewidth=2, markersize=8, label='Kruskal (并查集)')
        
        # 添加性能比例标注
        for i in range(len(node_counts_list)):
            if prim_dense_times[i] > 1e-9 and kruskal_dense_times[i] > 1e-9: # 避免除以零
                ratio = kruskal_dense_times[i] / prim_dense_times[i]
                plt.annotate(f'K/P={ratio:.2f}x', 
                            xy=(node_counts_list[i], (prim_dense_times[i] + kruskal_dense_times[i])/2),
                            xytext=(10, 0), textcoords='offset points',
                            fontsize=9, ha='left', bbox=dict(boxstyle="round,pad=0.3", fc="yellow", alpha=0.4))
    plt.xlabel('节点数量', fontsize=12)
    plt.ylabel('平均运行时间 (毫秒)', fontsize=12)
    plt.title('最优Prim与Kruskal在稠密图上的性能 (C++)', fontsize=14)
    plt.legend(fontsize=10)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir_path, "optimal_dense_comparison_cpp.png"), dpi=300, bbox_inches='tight')
    plt.close()
    
    # 图7: 总结图表 - 理论与实际比率
    plt.figure(figsize=(10, 7))
    sparse_ratios = []
    dense_ratios = []
    valid_nodes_sparse = []
    valid_nodes_dense = []

    for i in range(len(node_counts_list)):
        if results_dict['kruskal_union_find']['sparse'][i] > 1e-9 and not np.isnan(results_dict['prim_heap_optimized']['sparse'][i]):
            sparse_ratios.append(results_dict['prim_heap_optimized']['sparse'][i] / results_dict['kruskal_union_find']['sparse'][i])
            valid_nodes_sparse.append(node_counts_list[i])
        if results_dict['prim_heap_optimized']['dense'][i] > 1e-9 and not np.isnan(results_dict['kruskal_union_find']['dense'][i]):
            dense_ratios.append(results_dict['kruskal_union_find']['dense'][i] / results_dict['prim_heap_optimized']['dense'][i])
            valid_nodes_dense.append(node_counts_list[i])
            
    plt.subplot(2, 1, 1)
    plt.title('稀疏图: Prim/Kruskal 比值 (值>1 => Kruskal更快)', fontsize=12)
    if valid_nodes_sparse:
        plt.plot(valid_nodes_sparse, sparse_ratios, marker='o', linewidth=2)
        plt.axhline(y=1, color='r', linestyle='--', alpha=0.7)
        for i, ratio in enumerate(sparse_ratios):
            plt.annotate(f'{ratio:.2f}x', (valid_nodes_sparse[i], sparse_ratios[i]), textcoords="offset points", xytext=(0,5), ha='center', fontsize=9)
    plt.xlabel('节点数量', fontsize=11)
    plt.ylabel('性能比值 (Prim/Kruskal)', fontsize=11)
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.title('稠密图: Kruskal/Prim 比值 (值>1 => Prim更快)', fontsize=12)
    if valid_nodes_dense:
        plt.plot(valid_nodes_dense, dense_ratios, marker='s', color='green', linewidth=2)
        plt.axhline(y=1, color='r', linestyle='--', alpha=0.7)
        for i, ratio in enumerate(dense_ratios):
            plt.annotate(f'{ratio:.2f}x', (valid_nodes_dense[i], dense_ratios[i]), textcoords="offset points", xytext=(0,5), ha='center', fontsize=9)
    plt.xlabel('节点数量', fontsize=11)
    plt.ylabel('性能比值 (Kruskal/Prim)', fontsize=11)
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir_path, "theory_vs_actual_cpp.png"), dpi=300, bbox_inches='tight')
    plt.close()

    print(f"所有图表已保存到 {output_dir_path} 目录。")


# --- 主程序 ---
if __name__ == "__main__":
    experimental_data = run_all_experiments()
    if experimental_data:
        print("C++实验数据收集完成，开始生成图表...")
        plot_all_results(NODE_COUNTS, experimental_data, OUTPUT_DIR)
    else:
        print("由于实验数据收集失败，无法生成图表。") 