import numpy as np
import matplotlib.pyplot as plt

def place_tile(board, tile_number, row, col, special_row, special_col, size):
    """放置L型砖到棋盘上"""
    # 基本情况：2x2的棋盘
    if size == 2:
        for i in range(row, row + 2):
            for j in range(col, col + 2):
                if i != special_row or j != special_col:
                    board[i, j] = tile_number
        return tile_number + 1
    
    # 计算子棋盘大小和中心坐标
    half_size = size // 2
    center_row = row + half_size - 1
    center_col = col + half_size - 1
    
    # 确定特殊方格在哪个象限
    # 0: 左上, 1: 右上, 2: 左下, 3: 右下
    if special_row <= center_row and special_col <= center_col:
        quadrant = 0
    elif special_row <= center_row and special_col > center_col:
        quadrant = 1
    elif special_row > center_row and special_col <= center_col:
        quadrant = 2
    else:
        quadrant = 3
    
    # 放置中心L型砖
    current_tile = tile_number
    tile_number += 1
    
    # 通过对称性一次性处理四个子象限的特殊位置
    special_positions = [
        (center_row, center_col),           # 左上
        (center_row, center_col + 1),       # 右上
        (center_row + 1, center_col),       # 左下
        (center_row + 1, center_col + 1)    # 右下
    ]
    
    # 保持原有特殊方格的象限不变，更新其他三个象限
    for q in range(4):
        if q != quadrant:
            board[special_positions[q]] = current_tile
    
    # 准备四个子棋盘的特殊方格位置
    next_special = [
        (special_row, special_col) if quadrant == 0 else (center_row, center_col),
        (special_row, special_col) if quadrant == 1 else (center_row, center_col + 1),
        (special_row, special_col) if quadrant == 2 else (center_row + 1, center_col),
        (special_row, special_col) if quadrant == 3 else (center_row + 1, center_col + 1)
    ]
    
    # 递归处理四个子棋盘
    # 左上角子棋盘
    tile_number = place_tile(board, tile_number, row, col, 
                              next_special[0][0], next_special[0][1], half_size)
    
    # 右上角子棋盘
    tile_number = place_tile(board, tile_number, row, col + half_size, 
                              next_special[1][0], next_special[1][1], half_size)
    
    # 左下角子棋盘
    tile_number = place_tile(board, tile_number, row + half_size, col, 
                              next_special[2][0], next_special[2][1], half_size)
    
    # 右下角子棋盘
    tile_number = place_tile(board, tile_number, row + half_size, col + half_size, 
                              next_special[3][0], next_special[3][1], half_size)
    
    return tile_number

def solve_tiling(n, special_row=0, special_col=0):
    """解决铺砖问题
    
    参数:
    n -- 棋盘大小（边长，必须是2的幂）
    special_row, special_col -- 特殊方格的位置
    
    返回:
    棋盘数组，其中-1表示特殊方格，其他数字表示L型砖的编号
    """
    # 初始化棋盘
    board = np.zeros((n, n), dtype=int)
    board[special_row, special_col] = -1  # 标记特殊方格
    
    # 解决铺砖问题
    place_tile(board, 1, 0, 0, special_row, special_col, n)
    
    return board

def visualize_board(board):
    """可视化棋盘"""
    plt.figure(figsize=(10, 10))
    
    # 创建颜色映射
    cmap = plt.cm.get_cmap('viridis', np.max(board) + 2)
    
    # 特殊处理特殊方格（-1）
    masked_board = np.ma.masked_where(board == -1, board)
    plt.imshow(masked_board, cmap=cmap)
    
    # 标记特殊方格为黑色
    special_y, special_x = np.where(board == -1)
    plt.scatter(special_x, special_y, color='black', s=100, marker='s')
    
    # 添加网格和标签
    plt.grid(True, color='white', linestyle='-', linewidth=0.5)
    plt.title(f'棋盘铺砖结果 ({board.shape[0]}x{board.shape[0]})')
    plt.colorbar(label='L型砖编号')
    plt.tight_layout()
    plt.show()

# 示例用法
if __name__ == "__main__":
    print("铺砖问题求解器")
    
    # 用户输入
    try:
        n = int(input("输入棋盘大小(必须是2的幂): "))
        if n & (n-1) != 0:  # 检查是否为2的幂
            print("输入必须是2的幂!")
            exit(1)
        
        special_row = int(input("输入特殊方格的行位置: "))
        special_col = int(input("输入特殊方格的列位置: "))
        
        if special_row < 0 or special_row >= n or special_col < 0 or special_col >= n:
            print("特殊方格位置必须在棋盘范围内!")
            exit(1)
            
        # 解决铺砖问题
        board = solve_tiling(n, special_row, special_col)
        
        # 打印结果
        print("\n棋盘铺砖结果:")
        print(board)
        
        # 可视化结果
        visualize_board(board)
        
    except ValueError:
        print("请输入有效的整数!")