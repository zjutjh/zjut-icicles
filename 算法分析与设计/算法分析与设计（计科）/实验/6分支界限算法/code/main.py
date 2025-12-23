#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
分支界限算法演示主程序
Branch and Bound Algorithm Demonstration
"""

import sys
import time
from assignment_problem import AssignmentProblem, print_solution as print_assignment_solution
from tsp_problem import TSPProblem, print_tsp_solution

def run_assignment_tests():
    """运行任务分配问题测试"""
    print("=" * 60)
    print("                   任务分配问题测试")
    print("=" * 60)
    
    # 测试用例1：题目给出的例子
    print("\n测试用例1：题目示例 (4×4矩阵)")
    print("-" * 40)
    cost_matrix1 = [
        [9, 2, 7, 8],
        [6, 4, 3, 7],
        [5, 8, 1, 8],
        [7, 6, 9, 4]
    ]
    
    print("成本矩阵:")
    print("    Job1 Job2 Job3 Job4")
    workers = ['A', 'B', 'C', 'D']
    for i, row in enumerate(cost_matrix1):
        print(f"{workers[i]}:  {row}")
    
    start_time = time.time()
    solver1 = AssignmentProblem(cost_matrix1)
    min_cost1, assignment1 = solver1.solve()
    end_time = time.time()
    
    print_assignment_solution(cost_matrix1, assignment1, min_cost1)
    print(f"运行时间: {end_time - start_time:.4f} 秒")
    
    # 测试用例2：较小的矩阵
    print("\n\n测试用例2：3×3矩阵")
    print("-" * 40)
    cost_matrix2 = [
        [10, 19, 8],
        [15, 9, 7],
        [13, 12, 11]
    ]
    
    print("成本矩阵:")
    print("    Job1 Job2 Job3")
    for i, row in enumerate(cost_matrix2):
        print(f"{chr(65+i)}:  {row}")
    
    start_time = time.time()
    solver2 = AssignmentProblem(cost_matrix2)
    min_cost2, assignment2 = solver2.solve()
    end_time = time.time()
    
    print_assignment_solution(cost_matrix2, assignment2, min_cost2)
    print(f"运行时间: {end_time - start_time:.4f} 秒")
    
    # 测试用例3：对称矩阵
    print("\n\n测试用例3：对称矩阵 (4×4)")
    print("-" * 40)
    cost_matrix3 = [
        [1, 2, 3, 4],
        [2, 1, 4, 3],
        [3, 4, 1, 2],
        [4, 3, 2, 1]
    ]
    
    print("成本矩阵:")
    print("    Job1 Job2 Job3 Job4")
    for i, row in enumerate(cost_matrix3):
        print(f"{workers[i]}:  {row}")
    
    start_time = time.time()
    solver3 = AssignmentProblem(cost_matrix3)
    min_cost3, assignment3 = solver3.solve()
    end_time = time.time()
    
    print_assignment_solution(cost_matrix3, assignment3, min_cost3)
    print(f"运行时间: {end_time - start_time:.4f} 秒")

def run_tsp_tests():
    """运行TSP问题测试"""
    print("\n\n" + "=" * 60)
    print("                 旅行推销员问题测试")
    print("=" * 60)
    
    # 测试用例1：题目给出的例子
    print("\n测试用例1：题目示例 (4城市)")
    print("-" * 40)
    distance_matrix1 = [
        [0, 10, 15, 20],
        [10, 0, 35, 25],
        [15, 35, 0, 30],
        [20, 25, 30, 0]
    ]
    
    print("距离矩阵:")
    print("     0   1   2   3")
    for i, row in enumerate(distance_matrix1):
        print(f"{i}:  {row}")
    
    start_time = time.time()
    solver1 = TSPProblem(distance_matrix1)
    min_cost1, best_path1 = solver1.solve(0)
    end_time = time.time()
    
    print_tsp_solution(distance_matrix1, best_path1, min_cost1)
    print(f"运行时间: {end_time - start_time:.4f} 秒")
    
    # 测试用例2：较小的3城市问题
    print("\n\n测试用例2：3城市问题")
    print("-" * 40)
    distance_matrix2 = [
        [0, 10, 15],
        [10, 0, 20],
        [15, 20, 0]
    ]
    
    print("距离矩阵:")
    print("     0   1   2")
    for i, row in enumerate(distance_matrix2):
        print(f"{i}:  {row}")
    
    start_time = time.time()
    solver2 = TSPProblem(distance_matrix2)
    min_cost2, best_path2 = solver2.solve(0)
    end_time = time.time()
    
    print_tsp_solution(distance_matrix2, best_path2, min_cost2)
    print(f"运行时间: {end_time - start_time:.4f} 秒")
    
    # 测试用例3：5城市问题
    print("\n\n测试用例3：5城市问题")
    print("-" * 40)
    distance_matrix3 = [
        [0, 12, 10, 19, 8],
        [12, 0, 3, 7, 2],
        [10, 3, 0, 6, 20],
        [19, 7, 6, 0, 4],
        [8, 2, 20, 4, 0]
    ]
    
    print("距离矩阵:")
    print("     0   1   2   3   4")
    for i, row in enumerate(distance_matrix3):
        print(f"{i}:  {row}")
    
    start_time = time.time()
    solver3 = TSPProblem(distance_matrix3)
    min_cost3, best_path3 = solver3.solve(0)
    end_time = time.time()
    
    print_tsp_solution(distance_matrix3, best_path3, min_cost3)
    print(f"运行时间: {end_time - start_time:.4f} 秒")

def main():
    """主函数"""
    print("分支界限算法演示程序")
    print("Branch and Bound Algorithm Demonstration")
    print("作者: AI Assistant")
    print(f"Python版本: {sys.version}")
    
    # 运行任务分配问题测试
    run_assignment_tests()
    
    # 运行TSP问题测试
    run_tsp_tests()
    
    print("\n" + "=" * 60)
    print("                     测试完成")
    print("=" * 60)

if __name__ == "__main__":
    main() 