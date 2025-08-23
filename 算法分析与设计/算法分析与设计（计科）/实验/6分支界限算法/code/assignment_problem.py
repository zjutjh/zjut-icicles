#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务分配问题的分支界限算法实现
Assignment Problem using Branch and Bound Algorithm
"""

import heapq
import copy
from typing import List, Tuple, Optional

class AssignmentNode:
    """分支界限算法中的节点类"""
    
    def __init__(self, cost_matrix: List[List[int]], assigned: List[int], 
                 unassigned_workers: List[int], unassigned_jobs: List[int], 
                 cost: int, level: int, bound: float):
        self.cost_matrix = cost_matrix
        self.assigned = assigned  # assigned[i] = j 表示工人i被分配到工作j，-1表示未分配
        self.unassigned_workers = unassigned_workers
        self.unassigned_jobs = unassigned_jobs
        self.cost = cost  # 当前已分配的成本
        self.level = level  # 当前层级（已分配的工人数）
        self.bound = bound  # 下界
        
    def __lt__(self, other):
        """用于优先队列的比较"""
        return self.bound < other.bound

class AssignmentProblem:
    """任务分配问题求解器"""
    
    def __init__(self, cost_matrix: List[List[int]]):
        self.cost_matrix = cost_matrix
        self.n = len(cost_matrix)
        self.best_cost = float('inf')
        self.best_assignment = [-1] * self.n
        
    def calculate_bound(self, node: AssignmentNode) -> float:
        """计算当前节点的下界"""
        bound = node.cost
        
        # 对于未分配的工人，选择剩余工作中成本最小的
        for worker in node.unassigned_workers:
            if node.unassigned_jobs:
                min_cost = min(self.cost_matrix[worker][job] for job in node.unassigned_jobs)
                bound += min_cost
                
        return bound
    
    def is_promising(self, node: AssignmentNode) -> bool:
        """判断节点是否有希望找到更优解"""
        return node.bound < self.best_cost
    
    def solve(self) -> Tuple[int, List[int]]:
        """使用分支界限算法求解任务分配问题"""
        # 初始化根节点
        initial_assigned = [-1] * self.n
        initial_unassigned_workers = list(range(self.n))
        initial_unassigned_jobs = list(range(self.n))
        
        root = AssignmentNode(
            cost_matrix=self.cost_matrix,
            assigned=initial_assigned,
            unassigned_workers=initial_unassigned_workers,
            unassigned_jobs=initial_unassigned_jobs,
            cost=0,
            level=0,
            bound=0
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
                
            # 如果到达叶子节点（所有工人都已分配）
            if current.level == self.n:
                if current.cost < self.best_cost:
                    self.best_cost = current.cost
                    self.best_assignment = current.assigned.copy()
                continue
            
            # 为当前层级的工人分配每个可能的工作
            worker = current.level
            for job in current.unassigned_jobs:
                # 创建子节点
                new_assigned = current.assigned.copy()
                new_assigned[worker] = job
                
                new_unassigned_workers = [w for w in current.unassigned_workers if w != worker]
                new_unassigned_jobs = [j for j in current.unassigned_jobs if j != job]
                
                new_cost = current.cost + self.cost_matrix[worker][job]
                
                child = AssignmentNode(
                    cost_matrix=self.cost_matrix,
                    assigned=new_assigned,
                    unassigned_workers=new_unassigned_workers,
                    unassigned_jobs=new_unassigned_jobs,
                    cost=new_cost,
                    level=current.level + 1,
                    bound=0
                )
                child.bound = self.calculate_bound(child)
                
                # 如果子节点有希望，加入队列
                if self.is_promising(child):
                    heapq.heappush(priority_queue, child)
        
        print(f"探索了 {nodes_explored} 个节点")
        return self.best_cost, self.best_assignment

def print_solution(cost_matrix: List[List[int]], assignment: List[int], total_cost: int):
    """打印解决方案"""
    n = len(cost_matrix)
    print("\n=== 最优分配方案 ===")
    workers = ['A', 'B', 'C', 'D'][:n]
    
    for i in range(n):
        worker = workers[i] if i < len(workers) else f"Worker{i+1}"
        job = assignment[i] + 1
        cost = cost_matrix[i][assignment[i]]
        print(f"{worker} -> Job{job}, 成本: {cost}")
    
    print(f"总成本: {total_cost}")

def test_assignment_problem():
    """测试任务分配问题"""
    
    # 测试用例1：题目给出的例子
    print("=== 测试用例1：题目示例 ===")
    cost_matrix1 = [
        [9, 2, 7, 8],
        [6, 4, 3, 7],
        [5, 8, 1, 8],
        [7, 6, 9, 4]
    ]
    
    solver1 = AssignmentProblem(cost_matrix1)
    min_cost1, assignment1 = solver1.solve()
    print_solution(cost_matrix1, assignment1, min_cost1)
    
    # 测试用例2：较小的矩阵
    print("\n=== 测试用例2：3x3矩阵 ===")
    cost_matrix2 = [
        [10, 19, 8],
        [15, 9, 7],
        [13, 12, 11]
    ]
    
    solver2 = AssignmentProblem(cost_matrix2)
    min_cost2, assignment2 = solver2.solve()
    print_solution(cost_matrix2, assignment2, min_cost2)
    
    # 测试用例3：对称矩阵
    print("\n=== 测试用例3：对称矩阵 ===")
    cost_matrix3 = [
        [1, 2, 3, 4],
        [2, 1, 4, 3],
        [3, 4, 1, 2],
        [4, 3, 2, 1]
    ]
    
    solver3 = AssignmentProblem(cost_matrix3)
    min_cost3, assignment3 = solver3.solve()
    print_solution(cost_matrix3, assignment3, min_cost3)

if __name__ == "__main__":
    test_assignment_problem() 