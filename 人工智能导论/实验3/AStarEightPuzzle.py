import sys
import numpy as np
import time

# 定义状态类，表示八数码的当前状态


class State:
    def __init__(self, state, directionFlag=None, parent=None, f=0, depth=0):
        self.state = state  # 当前状态的矩阵表示
        self.direction = ['up', 'down', 'right', 'left']  # 移动方向
        if directionFlag:
            self.direction.remove(directionFlag)  # 移除不可行的方向
        self.parent = parent  # 父状态
        self.f = f  # 评估函数值
        self.depth = depth  # 当前深度

    def getDirection(self):
        return self.direction

    def setF(self, f):
        self.f = f
        return

    # 打印当前状态的矩阵
    def showInfo(self):
        for i in range(len(self.state)):
            for j in range(len(self.state)):
                print(self.state[i, j], end='  ')
            print("\n")
        print('->')
        return

    # 获取数字0的位置
    def getZeroPos(self):
        position = np.where(self.state == 0)
        return position

    # # 计算评估函数值，这里使用不在位节点数作为评估函数
    # def getFunctionValue(self):
    #     cur_node = self.state.copy()
    #     fin_node = self.answer.copy()
    #     dist = 0
    #     N = len(cur_node)

    #     for i in range(N):
    #         for j in range(N):
    #             if cur_node[i][j] != fin_node[i][j]:
    #                 dist += 1
    #     return dist + self.depth

    # # 通过计算曼哈顿举例来得到估价函数值
    # def getFunctionValue(self):
    #     cur_node = self.state.copy()
    #     fin_node = self.answer.copy()
    #     distance = 0
    #     N = len(cur_node)

    #     for i in range(N):
    #         for j in range(N):
    #             if cur_node[i][j] != 0:
    #                 target_i, target_j = np.where(fin_node == cur_node[i][j])
    #                 distance += abs(i - target_i) + abs(j - target_j)

    #     return distance + self.depth

    # 通过计算欧几里得距离来得到估价函数值
    def getFunctionValue(self):
        cur_node = self.state.copy()
        fin_node = self.answer.copy()
        distance = 0.0
        N = len(cur_node)

        for i in range(N):
            for j in range(N):
                if cur_node[i][j] != 0:
                    target_i, target_j = np.where(fin_node == cur_node[i][j])
                    distance += np.sqrt((i - target_i) ** 2 + (j - target_j) ** 2)

        return distance + self.depth

    # def getFunctionValue(self):
    #     return 0

    # 生成下一步可能的状态
    def nextStep(self):
        if not self.direction:
            return []
        subStates = []
        boarder = len(self.state) - 1
        # 获取0的位置
        x, y = self.getZeroPos()
        # 向左移动
        if 'left' in self.direction and y > 0:
            s = self.state.copy()
            tmp = s[x, y - 1]
            s[x, y - 1] = s[x, y]
            s[x, y] = tmp
            new_state = State(s, directionFlag='right',
                              parent=self, depth=self.depth + 1)
            new_state.setF(new_state.getFunctionValue())
            subStates.append(new_state)
        # 向上移动
        if 'up' in self.direction and x > 0:
            s = self.state.copy()
            tmp = s[x - 1, y]
            s[x - 1, y] = s[x, y]
            s[x, y] = tmp
            new_state = State(s, directionFlag='down',
                              parent=self, depth=self.depth + 1)
            new_state.setF(new_state.getFunctionValue())
            subStates.append(new_state)
        # 向下移动
        if 'down' in self.direction and x < boarder:
            s = self.state.copy()
            tmp = s[x + 1, y]
            s[x + 1, y] = s[x, y]
            s[x, y] = tmp
            new_state = State(s, directionFlag='up',
                              parent=self, depth=self.depth + 1)
            new_state.setF(new_state.getFunctionValue())
            subStates.append(new_state)
        # 向右移动
        if self.direction.count('right') and y < boarder:
            s = self.state.copy()
            tmp = s[x, y + 1]
            s[x, y + 1] = s[x, y]
            s[x, y] = tmp
            new_state = State(s, directionFlag='left',
                              parent=self, depth=self.depth + 1)
            new_state.setF(new_state.getFunctionValue())
            subStates.append(new_state)
        return subStates

    # 使用A*算法解决八数码问题
    def solve(self):
        # openList
        openTable = []
        # closeList
        closeTable = []
        openTable.append(self)

        while len(openTable) > 0:
            # 根据评估函数值排序openTable中的状态
            openTable.sort(key=compareNum)
            n = openTable.pop(0)
            # 将当前状态加入closeTable
            closeTable.append(n)
            # 获取下一步可能的状态
            next_states = n.nextStep()
            openTable.extend(next_states)

            for sub_state in next_states:
                path = []
                # 判断是否达到目标状态
                if (sub_state.state == sub_state.answer).all():
                    while sub_state.parent and sub_state.parent != originState:
                        path.append(sub_state.parent)
                        sub_state = sub_state.parent
                    path.reverse()
                    return path, openTable, closeTable
        else:
            return None, None


# 比较函数，用于排序
def compareNum(state):
    return state.f


def calculate_inversion_count(state):
    # 将二维状态展平为一维数组
    flat_state = state.flatten()
    flat_state = flat_state[flat_state != 0]

    # 计算逆序对数
    inversion_count = 0
    for i in range(len(flat_state)):
        for j in range(i + 1, len(flat_state)):
            if flat_state[i] > flat_state[j]:
                inversion_count += 1

    return inversion_count


def is_solvable(start_state, goal_state):
    # 计算初始状态和目标状态的逆序对数
    start_inversions = calculate_inversion_count(start_state)
    goal_inversions = calculate_inversion_count(goal_state)

    # 判断奇偶性
    return (start_inversions % 2 == goal_inversions % 2), start_inversions % 2, goal_inversions % 2


if __name__ == '__main__':
    originState = State(np.array([[2, 3, 1],
                                  [8, 0, 4],
                                  [7, 6, 5]]))
    State.answer = np.array([[2, 1, 3],
                            [8, 0, 4],
                            [7, 6, 5]])

    # s1 = State(state=originState.state)
    # path, openTable, closeTable = s1.solve()
    # if path:
    #     for node in path:
    #         node.showInfo()
    #     print(State.answer)
    #     print("Total steps is %d" % len(path))
    #     print("拓展节点数:", len(closeTable))
    #     print("生成节点数:", len(openTable))
    #     print("Open表内容:")
    #     for i, node in enumerate(openTable[:5]):  # 打印前5个状态
    #         print("状态", i + 1, ":")
    #         node.showInfo()
    #     print("Close表内容:")
    #     for i, node in enumerate(closeTable[:5]):  # 打印前5个状态
    #         print("状态", i + 1, ":")
    #         node.showInfo()
    solvable, start_inversions, goal_inversions = is_solvable(
        originState.state, State.answer)
    print("这个八数码问题是否有解：", solvable, "\nstart_inversions % 2 =", \
          start_inversions % 2, "\ngoal_inversions % 2 =", goal_inversions % 2)
