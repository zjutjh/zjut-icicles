#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
旅行推销员问题(TSP)的分支界限算法实现
Traveling Salesman Problem using Branch and Bound Algorithm
"""

import heapq
import copy
from typing import List, Tuple, Optional

class TSPNode:
    """分支界限算法中的节点类"""
    
    def __init__(self, path: List[int], cost: int, bound: float, level: int, visited: List[bool]):
        self.path = path  # 当前路径
        self.cost = cost  # 当前路径的成本
        self.bound = bound  # 下界
        self.level = level  # 当前层级（已访问的城市数）
        self.visited = visited  # 访问标记数组
        
    def __lt__(self, other):
        """用于优先队列的比较"""
        return self.bound < other.bound

class TSPProblem:
    """旅行推销员问题求解器"""
    
    def __init__(self, distance_matrix: List[List[int]]):
        self.distance_matrix = distance_matrix
        self.n = len(distance_matrix)
        self.best_cost = float('inf')
        self.best_path = []
        
    def calculate_bound(self, node: TSPNode) -> float:
        """计算当前节点的下界"""
        bound = node.cost
        
        # 对于未访问的城市，计算最小边权重的和
        # 使用最小生成树的近似方法
        
        # 找到当前路径中最后一个城市
        if node.level > 0:
            last_city = node.path[-1]
            # 从最后一个城市到未访问城市的最小距离
            min_cost_from_last = float('inf')
            for i in range(self.n):
                if not node.visited[i] and self.distance_matrix[last_city][i] < min_cost_from_last:
                    min_cost_from_last = self.distance_matrix[last_city][i]
            if min_cost_from_last != float('inf'):
                bound += min_cost_from_last
        
        # 对于每个未访问的城市，加上其最小出边
        unvisited_count = 0
        for i in range(self.n):
            if not node.visited[i]:
                unvisited_count += 1
                min_edge = float('inf')
                for j in range(self.n):
                    if i != j and self.distance_matrix[i][j] < min_edge:
                        min_edge = self.distance_matrix[i][j]
                if min_edge != float('inf'):
                    bound += min_edge * 0.5  # 除以2是因为每条边被计算了两次
        
        # 如果需要回到起点，加上回到起点的成本
        if node.level == self.n - 1 and len(node.path) > 0:
            bound += self.distance_matrix[node.path[-1]][0]
            
        return bound
    
    def is_promising(self, node: TSPNode) -> bool:
        """判断节点是否有希望找到更优解"""
        return node.bound < self.best_cost
    
    def solve(self, start_city: int = 0) -> Tuple[int, List[int]]:
        """使用分支界限算法求解TSP问题"""
        # 初始化根节点
        initial_visited = [False] * self.n
        initial_visited[start_city] = True
        
        root = TSPNode(
            path=[start_city],
            cost=0,
            bound=0,
            level=1,
            visited=initial_visited
        )
        root.bound = self.calculate_bound(root)
        
        # 优先队列（最小堆）
        priority_queue = [root]
        nodes_explored = 0
        
        while priority_queue:
            current = heapq.heappop(priority_queue)
            nodes_explored += 1
            
            # 如果当前节点的下界大于等于最优解，剪枝
            if not self.is_promising(current):
                continue
                
            # 如果访问了所有城市
            if current.level == self.n:
                # 计算回到起点的总成本
                total_cost = current.cost + self.distance_matrix[current.path[-1]][start_city]
                if total_cost < self.best_cost:
                    self.best_cost = total_cost
                    self.best_path = current.path + [start_city]
                continue
            
            # 为下一个城市生成所有可能的选择
            for next_city in range(self.n):
                if not current.visited[next_city]:
                    # 创建子节点
                    new_path = current.path + [next_city]
                    new_cost = current.cost + self.distance_matrix[current.path[-1]][next_city]
                    new_visited = current.visited.copy()
                    new_visited[next_city] = True
                    
                    child = TSPNode(
                        path=new_path,
                        cost=new_cost,
                        bound=0,
                        level=current.level + 1,
                        visited=new_visited
                    )
                    child.bound = self.calculate_bound(child)
                    
                    # 如果子节点有希望，加入队列
                    if self.is_promising(child):
                        heapq.heappush(priority_queue, child)
        
        print(f"探索了 {nodes_explored} 个节点")
        return self.best_cost, self.best_path

def print_tsp_solution(distance_matrix: List[List[int]], path: List[int], total_cost: int):
    """打印TSP解决方案"""
    print("\n=== 最优路径 ===")
    print("路径:", " -> ".join(map(str, path)))
    
    print("\n=== 路径详情 ===")
    for i in range(len(path) - 1):
        from_city = path[i]
        to_city = path[i + 1]
        distance = distance_matrix[from_city][to_city]
        print(f"城市 {from_city} -> 城市 {to_city}: 距离 = {distance}")
    
    print(f"\n总距离: {total_cost}")

def test_tsp_problem():
    """测试TSP问题"""
    
    # 测试用例1：题目给出的例子
    print("=== 测试用例1：题目示例 ===")
    distance_matrix1 = [
        [0, 10, 15, 20],
        [10, 0, 35, 25],
        [15, 35, 0, 30],
        [20, 25, 30, 0]
    ]
    
    solver1 = TSPProblem(distance_matrix1)
    min_cost1, best_path1 = solver1.solve(0)
    print_tsp_solution(distance_matrix1, best_path1, min_cost1)
    
    # 测试用例2：较小的3城市问题
    print("\n=== 测试用例2：3城市问题 ===")
    distance_matrix2 = [
        [0, 10, 15],
        [10, 0, 20],
        [15, 20, 0]
    ]
    
    solver2 = TSPProblem(distance_matrix2)
    min_cost2, best_path2 = solver2.solve(0)
    print_tsp_solution(distance_matrix2, best_path2, min_cost2)
    
    # 测试用例3：5城市问题
    print("\n=== 测试用例3：5城市问题 ===")
    distance_matrix3 = [
        [0, 12, 10, 19, 8],
        [12, 0, 3, 7, 2],
        [10, 3, 0, 6, 20],
        [19, 7, 6, 0, 4],
        [8, 2, 20, 4, 0]
    ]
    
    solver3 = TSPProblem(distance_matrix3)
    min_cost3, best_path3 = solver3.solve(0)
    print_tsp_solution(distance_matrix3, best_path3, min_cost3)

if __name__ == "__main__":
    test_tsp_problem() 