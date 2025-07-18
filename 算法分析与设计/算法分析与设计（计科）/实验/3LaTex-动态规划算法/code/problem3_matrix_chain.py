"""
问题3：矩阵乘法最优结合
求解矩阵连乘的最佳计算顺序
"""

import sys
import numpy as np

def matrix_multiply_cost(p, i, j):
    """计算一次矩阵乘法的代价"""
    return p[i-1] * p[i] * p[j]

def matrix_chain_brute_force(p, i, j):
    """
    递归穷举法求解矩阵连乘的最优计算顺序
    p是矩阵维度序列，p[i-1] x p[i]表示第i个矩阵的维度
    时间复杂度：O(2^n)，其中n是矩阵数量
    """
    if i == j:
        return 0  # 只有一个矩阵，无需乘法
    
    min_cost = float('inf')
    for k in range(i, j):
        # 计算在k处分割的代价
        cost = (matrix_chain_brute_force(p, i, k) + 
                matrix_chain_brute_force(p, k+1, j) + 
                p[i-1] * p[k] * p[j])
        if cost < min_cost:
            min_cost = cost
    
    return min_cost

def matrix_chain_dp(p):
    """
    动态规划法求解矩阵连乘的最优计算顺序
    p是矩阵维度序列，p[i] x p[i+1]表示第i+1个矩阵的维度
    时间复杂度：O(n^3)，其中n是矩阵数量
    """
    n = len(p) - 1  # 矩阵数量
    
    # 创建dp表，m[i][j]表示计算第i个矩阵到第j个矩阵的最小乘法次数
    m = [[0] * (n + 1) for _ in range(n + 1)]
    
    # 创建s表，s[i][j]记录最优分割点
    s = [[0] * (n + 1) for _ in range(n + 1)]
    
    # 计算链长为l的所有子问题
    for l in range(2, n + 1):
        for i in range(1, n - l + 2):
            j = i + l - 1
            m[i][j] = float('inf')
            for k in range(i, j):
                # 计算在k处分割的代价
                cost = m[i][k] + m[k+1][j] + p[i-1] * p[k] * p[j]
                if cost < m[i][j]:
                    m[i][j] = cost
                    s[i][j] = k
    
    return m, s

def print_optimal_parens(s, i, j):
    """打印最优括号化方案"""
    if i == j:
        print(f"A{i}", end="")
    else:
        print("(", end="")
        print_optimal_parens(s, i, s[i][j])
        print_optimal_parens(s, s[i][j] + 1, j)
        print(")", end="")

def print_dp_table(p, m):
    """打印动态规划表"""
    n = len(p) - 1
    print("动态规划表 m[i,j]:")
    
    # 打印表头
    print("     ", end="")
    for j in range(1, n + 1):
        print(f"j={j:<4}", end="")
    print()
    
    # 打印表内容
    for i in range(1, n + 1):
        print(f"i={i:<2} ", end="")
        for j in range(1, n + 1):
            if j >= i:
                print(f"{m[i][j]:<7}", end="")
            else:
                print("       ", end="")
        print()

def main():
    # 例1：计算(AB)C和A(BC)的操作次数
    A = (10, 30)  # 10x30
    B = (30, 5)   # 30x5
    C = (5, 60)   # 5x60
    
    # (AB)C
    AB_cost = A[0] * A[1] * B[1]  # 10 * 30 * 5 = 1500
    AB_result = (A[0], B[1])      # 10x5
    ABC_cost = AB_cost + AB_result[0] * AB_result[1] * C[1]  # 1500 + 10 * 5 * 60 = 4500
    
    # A(BC)
    BC_cost = B[0] * B[1] * C[1]  # 30 * 5 * 60 = 9000
    BC_result = (B[0], C[1])      # 30x60
    ABC_cost2 = BC_cost + A[0] * A[1] * BC_result[1]  # 9000 + 10 * 30 * 60 = 27000
    
    print("例1：计算(AB)C和A(BC)的操作次数")
    print(f"矩阵A: {A[0]}x{A[1]}")
    print(f"矩阵B: {B[0]}x{B[1]}")
    print(f"矩阵C: {C[0]}x{C[1]}")
    print(f"(AB)C 操作次数: {ABC_cost}")
    print(f"A(BC) 操作次数: {ABC_cost2}")
    print(f"最优方案: {'(AB)C' if ABC_cost < ABC_cost2 else 'A(BC)'}")
    print()
    
    # 例2：使用动态规划求解6个矩阵的最优计算顺序
    p = [30, 35, 15, 5, 10, 20, 25]
    m, s = matrix_chain_dp(p)
    
    print("例2：6个矩阵的最优计算顺序")
    print(f"矩阵维度: {p}")
    print(f"最小乘法次数: {m[1][len(p)-1]}")
    print("最优括号化方案: ", end="")
    print_optimal_parens(s, 1, len(p)-1)
    print("\n")
    
    # 完整打印动态规划表
    n = len(p) - 1
    print("动态规划表 m[i,j]:")
    print("     ", end="")
    for j in range(1, n + 1):
        print(f"j={j:<4}", end="")
    print()
    
    for i in range(1, n + 1):
        print(f"i={i:<2} ", end="")
        for j in range(1, n + 1):
            if j >= i:
                print(f"{m[i][j]:<7}", end="")
            else:
                print("       ", end="")
        print()

if __name__ == "__main__":
    main() 