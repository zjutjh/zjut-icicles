import numpy as np
import random
import matplotlib.pyplot as plt
import copy
import time

# 各个城市的坐标
# 10个城市坐标
# City_Map =[[0.4,0.4439],[0.2439,0.1463],[0.1707,0.2293],[0.2293,0.761],[0.5171,0.9414],[0.8732,0.6536],[0.6878,0.5219],[0.8488,0.3609],[0.6683,0.2536],[0.6195,0.2634]]
# 20个城市坐标
# City_Map =[[41,94],[37,84],[54,67],[25,62],[7,64],[2,99],[68,58],[71,44],[54,62],[83,69],[64,60],[18,54],[22,60],[83,46],[91,38],[25,38],[24,42],[58,69],[71,71],[74,78]]

# City_Map = 100 * np.random.rand(10, 2)#随机产生10个城市坐标

# 34个省会城市及像素坐标表
City_Map = [[100, 211], [187, 265], [201, 214], [187, 158], [221, 142], [202, 165], [258, 121], [352, 66], [346, 106], [336, 106], [290, 127], [297, 135],
            [278, 147], [296, 158], [274, 177], [265, 148], [239, 182], [302, 203], [
                316, 199], [334, 206], [325, 215], [293, 233], [280, 216],
            [271, 238], [221, 253], [233, 287], [275, 285], [322, 254], [250, 315], [277, 293], [286, 290], [342, 263], [220, 226], [104, 77]]

DNA_SIZE = len(City_Map)  # 编码长度
POP_SIZE = 80  # 种群大小
CROSS_RATE = 0.8  # 交叉率
MUTA_RATE = 0.2  # 变异率
Iterations = 200  # 迭代次数


def distance(DNA):  # 根据个体染色体基因的路线计算距离
    dis = 0
    temp = City_Map[DNA[0]]
    for i in DNA[1:]:
        dis = dis + ((City_Map[i][0]-temp[0])**2 +
                     (City_Map[i][1]-temp[1])**2)**0.5
        temp = City_Map[i]
    return dis+((temp[0]-City_Map[DNA[0]][0])**2+(temp[1]-City_Map[DNA[0]][1])**2)**0.5


def getfitness(pop):  # 计算种群适应度，这里适应度用距离的倒数表示
    temp = []
    for i in range(len(pop)):
        temp.append(1/(distance(pop[i])))
    return temp-np.min(temp)


def select(pop, fitness):    # 根据适应度比例分配选择概率，以赌轮盘的形式，适应度越大的个体被选中的概率越大
    s = fitness.sum()
    temp = np.random.choice(np.arange(len(pop)),
                            size=POP_SIZE, replace=True, p=(fitness/s))
    p = []
    for i in temp:
        p.append(pop[i])
    return p


def mutation(DNA, MUTA_RATE):  # 进行变异：两点互换
    if np.random.rand() < MUTA_RATE:  # 以MUTA_RATE的概率进行变异
        mutate_point1 = np.random.randint(0, DNA_SIZE)  # 随机产生一个实数，代表要变异基因的位置
        mutate_point2 = np.random.randint(0, DNA_SIZE)  # 随机产生一个实数，代表要变异基因的位置
        while (mutate_point1 == mutate_point2):  # 保证2个所选位置不相等
            mutate_point2 = np.random.randint(0, DNA_SIZE)
        # 2个所选位置进行互换
        DNA[mutate_point1], DNA[mutate_point2] = DNA[mutate_point2], DNA[mutate_point1]


def reverse_mutation(DNA, MUTA_RATE):  # 进行变异：逆转变异
    if np.random.rand() < MUTA_RATE:  # 以MUTA_RATE的概率进行变异
        mutate_point1 = np.random.randint(0, DNA_SIZE)  # 随机产生一个实数，代表要变异基因的位置
        mutate_point2 = np.random.randint(0, DNA_SIZE)  # 随机产生一个实数，代表要变异基因的位置
        while (mutate_point1 == mutate_point2):  # 保证2个所选位置不相等
            mutate_point2 = np.random.randint(0, DNA_SIZE)
        if mutate_point1 > mutate_point2:  # 确保mutate_point1小于mutate_point2
            mutate_point1, mutate_point2 = mutate_point2, mutate_point1
        # 逆转mutate_point1和mutate_point2之间的基因
        DNA[mutate_point1:mutate_point2] = DNA[mutate_point1:mutate_point2][::-1]


def insertion_mutation(DNA, MUTA_RATE):  # 进行变异：插入变异
    if np.random.rand() < MUTA_RATE:  # 以MUTA_RATE的概率进行变异
        mutate_point1 = np.random.randint(0, DNA_SIZE)  # 随机产生一个实数，代表要变异基因的位置
        mutate_point2 = np.random.randint(0, DNA_SIZE)  # 随机产生一个实数，代表要插入基因的位置
        while (mutate_point1 == mutate_point2):  # 保证2个所选位置不相等
            mutate_point2 = np.random.randint(0, DNA_SIZE)
        gene = DNA[mutate_point1]  # 提取要插入的基因
        DNA = np.insert(DNA, mutate_point2, gene)  # 在mutate_point2位置插入基因
        if mutate_point2 < mutate_point1:  # 如果插入位置在提取基因位置之前，提取基因位置需要加1
            mutate_point1 += 1
        DNA = np.delete(DNA, mutate_point1)  # 删除原来位置的基因


def crossmuta(pop, CROSS_RATE):  # 交叉变异
    new_pop = []
    for i in range(len(pop)):  # 遍历种群中的每一个个体，将该个体作为父代
        n = np.random.rand()
        if n >= CROSS_RATE:  # 大于交叉概率时不发生交叉变异，该子代直接进入下一代
            temp = pop[i].copy()
            new_pop.append(temp)

        if n < CROSS_RATE:  # 小于交叉概率时发生交叉变异
            list1 = pop[i].copy()  # 选取交叉前的一个父代
            # 选取种群中另一个个体为交叉前的父代
            list2 = pop[np.random.randint(POP_SIZE)].copy()
            status = True
            while status:
                k1 = random.randint(0, len(list1) - 1)  # 随机产生交叉位k1
                k2 = random.randint(0, len(list2) - 1)  # 随机产生交叉位k2
                if k1 < k2:  # 保证交叉位 k1 < k2
                    status = False

            k11 = k1
            fragment1 = list1[k1: k2]  # 父代1的交叉区域的基因串
            fragment2 = list2[k1: k2]  # 父代2的交叉区域的基因串
            # 交叉区域的基因串互换
            list1[k1: k2] = fragment2
            list2[k1: k2] = fragment1

            # del list1[k1: k2]
            # 请补充代码，解决由于交叉引起的同一个染色体中出现相同基因的问题，即部分匹配交叉策略
            # 部分匹配交叉策略
            # for i in range(len(list1)):
            #     if i < k1 or i >= k2:
            #         while list1[i] in fragment2:
            #             index = fragment2.index(list1[i])
            #             list1[i] = fragment1[index]

            #         while list2[i] in fragment1:
            #             index = fragment1.index(list2[i])
            #             list2[i] = fragment2[index]
            # offspring1 = []

            # temp = offspring1.copy()#temp为交叉后的一个子代

            # temp = pop[i].copy()  # 加了交叉操作后，删除该行代码
            # mutation(list1, MUTA_RATE)  # 变异
            reverse_mutation(list1, MUTA_RATE)  # 逆转变异

            new_pop.append(list1)

    return new_pop


def print_info(pop):  # 用于输出结果
    fitness = getfitness(pop)
    maxfitness = np.argmax(fitness)  # 得到种群中最大适应度个体的索引
    # 打印结果
    print("最优个体：", pop[maxfitness])
    print("最短距离：", distance(pop[maxfitness]))
    # 按最优结果顺序把地图上的点加入到best_map列表中
    best_map = []
    for i in pop[maxfitness]:
        best_map.append(City_Map[i])
    best_map.append(City_Map[pop[maxfitness][0]])
    X = np.array((best_map))[:, 0]
    Y = np.array((best_map))[:, 1]
    # 绘制地图以及路线
    plt.figure()
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.scatter(X, Y)
    for dot in range(len(X)-1):
        plt.annotate(pop[maxfitness][dot], xy=(
            X[dot], Y[dot]), xytext=(X[dot], Y[dot]))
    plt.annotate('start', xy=(X[0], Y[0]), xytext=(X[0]+1, Y[0]))
    plt.plot(X, Y)


# if __name__ == "__main__":#主循环
#     time_start = time.perf_counter() # 记录开始时间
#     #生成初代种群pop
#     pop = []
#     list = list(range(DNA_SIZE))
#     for i in range(POP_SIZE):
#         random.shuffle(list)
#         l = list.copy()
#         pop.append(l)
#     best_dis= []
#     #进行选择，交叉，变异，并把每代的最优个体保存在best_dis中
#     for i in range(Iterations):  # 迭代N代
#         pop = crossmuta(pop, CROSS_RATE)
#         fitness = getfitness(pop)
#         maxfitness = np.argmax(fitness)
#         best_dis.append(distance(pop[maxfitness]))
#         pop = select(pop, fitness)  # 选择生成新的种群
#
#     print_info(pop)#打印信息
#
#     print('每次迭代的最短路径：',best_dis)
#     time_end = time.perf_counter()  # 记录结束时间
#     time_sum = time_end - time_start  # 计算的时间差为程序的执行时间，单位为秒/s
#     print('算法运行时间(单位为秒):',time_sum)
# #画图
# plt.figure()
# plt.plot(range(Iterations),best_dis)
# plt.show()
# plt.close()

if __name__ == "__main__":
    best_values = []  # 用于记录每次运行的最好值
    worst_values = []  # 用于记录每次运行的最差值
    avg_values = []  # 用于记录每次运行的平均值
    run_times = []  # 用于记录每次运行的时间
    best_paths = []  # 用于记录每次迭代的最好路径
    best_pop = []  # 用于记录每次运行的最好种群

    for _ in range(10):  # 独立运行10次
        time_start = time.perf_counter()  # 记录开始时间

        # 生成初代种群pop
        pop = []
        my_list = list(range(DNA_SIZE))
        for i in range(POP_SIZE):
            random.shuffle(my_list)
            l = my_list.copy()
            pop.append(l)

        best_dis = []  # 用于记录每次迭代的最短路径

        # 进行选择，交叉，变异，并把每代的最优个体保存在best_dis中
        for i in range(Iterations):  # 迭代200个epoch
            pop = crossmuta(pop, CROSS_RATE)
            fitness = getfitness(pop)  # 计算种群适应度
            maxfitness = np.argmax(fitness)
            best_dis.append(distance(pop[maxfitness]))
            pop = select(pop, fitness)  # 选择生成新的种群

        print_info(pop)  # 打印信息

        time_end = time.perf_counter()  # 记录结束时间
        time_sum = time_end - time_start  # 计算的时间差为程序的执行时间，单位为秒/s

        # 记录本次运行的结果
        best_values.append(min(best_dis))
        worst_values.append(max(best_dis))
        avg_values.append(sum(best_dis) / len(best_dis))
        run_times.append(time_sum)
        best_paths.append(best_dis)  # 保存最好路径
        best_pop.append(pop[maxfitness])  # 保存最好种群

        print('--------------------------------------')

    print('--------------------------------------')

    # 输出10次运行的结果
    print('10次运行中种群的最好值：', min(best_values))
    print('10次运行中种群的最差值：', max(worst_values))
    print('10次运行中种群的平均值：', sum(avg_values) / len(avg_values))
    print('平均运行时间(单位为秒)：', sum(run_times) / len(run_times))

    # 找到最好的路径
    best_index = best_values.index(min(best_values))
    best_path = best_paths[best_index]
    best_pop_path = best_pop[best_index]

    # 输出最好的种群路径
    print('最好的种群路径：', best_pop_path)

    # 画图
    plt.figure()
    plt.plot(range(Iterations), best_path)
    plt.show()
    plt.close()
