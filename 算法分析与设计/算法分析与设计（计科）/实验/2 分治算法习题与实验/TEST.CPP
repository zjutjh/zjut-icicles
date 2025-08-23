
#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

// 全局变量，存储棋盘和砖块的编号
vector<vector<int>> board;
int tileNumber = 1;

// 放置L型砖的函数
void placeTile(int row, int col, int specialRow, int specialCol, int size) {
    // 如果只剩下一个2x2的棋盘，直接放置L型砖
    if (size == 2) {
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                if (row + i != specialRow || col + j != specialCol) {
                    board[row + i][col + j] = tileNumber;
                }
            }
        }
        tileNumber++;
        return;
    }
    
    int halfSize = size / 2;
    int centerRow = row + halfSize - 1;
    int centerCol = col + halfSize - 1;
    
    // 确定特殊方格在哪个象限
    int quadrant;
    if (specialRow <= centerRow && specialCol <= centerCol) {
        quadrant = 0; // 左上角
    } else if (specialRow <= centerRow && specialCol > centerCol) {
        quadrant = 1; // 右上角
    } else if (specialRow > centerRow && specialCol <= centerCol) {
        quadrant = 2; // 左下角
    } else {
        quadrant = 3; // 右下角
    }
    
    // 放置中心L型砖
    int currentTile = tileNumber++;
    
    // 右上角
    if (quadrant != 1) board[centerRow][centerCol + 1] = currentTile;
    // 左下角
    if (quadrant != 2) board[centerRow + 1][centerCol] = currentTile;
    // 右下角
    if (quadrant != 3) board[centerRow + 1][centerCol + 1] = currentTile;
    // 左上角
    if (quadrant != 0) board[centerRow][centerCol] = currentTile;
    
    // 递归处理四个子棋盘
    
    // 左上角子棋盘
    int nextSpecialRow = (quadrant == 0) ? specialRow : centerRow;
    int nextSpecialCol = (quadrant == 0) ? specialCol : centerCol;
    placeTile(row, col, nextSpecialRow, nextSpecialCol, halfSize);
    
    // 右上角子棋盘
    nextSpecialRow = (quadrant == 1) ? specialRow : centerRow;
    nextSpecialCol = (quadrant == 1) ? specialCol : centerCol + 1;
    placeTile(row, col + halfSize, nextSpecialRow, nextSpecialCol, halfSize);
    
    // 左下角子棋盘
    nextSpecialRow = (quadrant == 2) ? specialRow : centerRow + 1;
    nextSpecialCol = (quadrant == 2) ? specialCol : centerCol;
    placeTile(row + halfSize, col, nextSpecialRow, nextSpecialCol, halfSize);
    
    // 右下角子棋盘
    nextSpecialRow = (quadrant == 3) ? specialRow : centerRow + 1;
    nextSpecialCol = (quadrant == 3) ? specialCol : centerCol + 1;
    placeTile(row + halfSize, col + halfSize, 
              nextSpecialRow, nextSpecialCol, halfSize);
}

// 打印棋盘的函数
void printBoard(int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (board[i][j] == -1) {
                cout << "S\t"; // 特殊方格
            } else {
                cout << board[i][j] << "\t";
            }
        }
        cout << endl;
    }
}

int main() {
    int n;
    cout << "输入棋盘大小(必须是2的幂): ";
    cin >> n;
    
    // 初始化棋盘
    board.resize(n, vector<int>(n, 0));
    
    // 设置特殊方格的位置（这里设为左上角，可以根据需要修改）
    int specialRow = 0, specialCol = 0;
    cout << "输入特殊方格的位置(行 列): ";
    cin >> specialRow >> specialCol;
    
    // 标记特殊方格
    board[specialRow][specialCol] = -1;
    
    // 解决铺砖问题
    placeTile(0, 0, specialRow, specialCol, n);
    
    // 打印结果
    printBoard(n);
    
    return 0;
}