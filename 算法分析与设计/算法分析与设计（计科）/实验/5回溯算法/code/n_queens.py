def solve_n_queens(n):
    board = [['.' for _ in range(n)] for _ in range(n)]
    solutions = []
    
    def backtrack(row):
        if row == n:
            # 找到一个解，保存它
            solutions.append([''.join(row) for row in board])
            return
        
        for col in range(n):
            if is_valid(row, col):
                # 放置皇后
                board[row][col] = 'Q'
                # 递归到下一行
                backtrack(row + 1)
                # 回溯
                board[row][col] = '.'
    
    def is_valid(row, col):
        # 检查列冲突
        for i in range(row):
            if board[i][col] == 'Q':
                return False
        
        # 检查左上斜线冲突
        i, j = row - 1, col - 1
        while i >= 0 and j >= 0:
            if board[i][j] == 'Q':
                return False
            i -= 1
            j -= 1
        
        # 检查右上斜线冲突
        i, j = row - 1, col + 1
        while i >= 0 and j < n:
            if board[i][j] == 'Q':
                return False
            i -= 1
            j += 1
        
        return True
    
    backtrack(0)
    return solutions

def print_solutions(solutions):
    for i, solution in enumerate(solutions):
        print(f"解决方案 {i+1}:")
        for row in solution:
            print(row)
        print()

# 测试用例
if __name__ == "__main__":
    n = 4  # 4皇后问题
    solutions = solve_n_queens(n)
    print(f"{n}皇后问题共有 {len(solutions)} 种解决方案：")
    print_solutions(solutions) 