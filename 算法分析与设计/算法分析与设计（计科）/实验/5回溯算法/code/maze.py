def solve_maze(maze):
    # 迷宫尺寸
    n = len(maze)
    m = len(maze[0])
    
    # 创建一个二维数组来存储路径
    solution = [[0 for _ in range(m)] for _ in range(n)]
    
    # 定义移动方向：右、下、左、上
    move_x = [0, 1, 0, -1]
    move_y = [1, 0, -1, 0]
    
    # 创建访问标记数组，避免重复访问
    visited = [[False for _ in range(m)] for _ in range(n)]
    
    if not solve_maze_util(maze, 0, 0, solution, move_x, move_y, n, m, visited):
        print("没有找到通往出口的路径")
        return None
    
    return solution

def solve_maze_util(maze, x, y, solution, move_x, move_y, n, m, visited):
    # 达到右下角，找到解
    if x == n - 1 and y == m - 1 and maze[x][y] == 0:
        solution[x][y] = 1
        return True
    
    # 检查当前单元格是否有效且未访问
    if is_safe(maze, x, y, n, m) and not visited[x][y]:
        # 标记当前单元格为已访问
        visited[x][y] = True
        
        # 标记当前单元格为解决方案路径的一部分
        solution[x][y] = 1
        
        # 尝试四个方向移动
        for i in range(4):
            next_x = x + move_x[i]
            next_y = y + move_y[i]
            
            if solve_maze_util(maze, next_x, next_y, solution, move_x, move_y, n, m, visited):
                return True
        
        # 如果没有方向可行，回溯
        solution[x][y] = 0
        return False
    
    return False

def is_safe(maze, x, y, n, m):
    # 检查是否在迷宫内且是通路
    return 0 <= x < n and 0 <= y < m and maze[x][y] == 0

def print_solution(solution):
    if solution is None:
        return
    
    n = len(solution)
    m = len(solution[0])
    
    print("路径解决方案（1表示路径）：")
    for i in range(n):
        for j in range(m):
            print(solution[i][j], end=" ")
        print()

# 测试用例
if __name__ == "__main__":
    # 修改迷宫，使其有一条可行路径
    maze = [
        [0, 1, 0],
        [0, 0, 0],  # 中间位置改为0，使其有路径
        [1, 0, 0]
    ]
    
    print("迷宫（0表示通路，1表示墙）：")
    for row in maze:
        print(" ".join(map(str, row)))
    print()
    
    solution = solve_maze(maze)
    
    if solution:
        print_solution(solution)
    else:
        print("未找到路径") 