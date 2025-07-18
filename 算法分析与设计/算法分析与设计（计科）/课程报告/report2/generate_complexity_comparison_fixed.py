import matplotlib.pyplot as plt
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'Microsoft YaHei', 'sans-serif']  # 增加备用字体
plt.rcParams['axes.unicode_minus'] = False  # 正确显示负号

# 定义复杂度函数
def constant(n):
    return np.ones_like(n)

def logarithmic(n):
    return np.log2(n)

def linear(n):
    return n

def linearithmic(n):
    return n * np.log2(n)

def quadratic(n):
    return n ** 2

def exponential(n):
    # 限制指数函数的增长，以便在图表上可见
    return 2 ** np.minimum(n, 10)  # 限制最大值

# 创建数据点
n = np.linspace(1, 10, 100)  # 输入规模从1到10

# 计算各种复杂度
y_constant = constant(n)
y_logarithmic = logarithmic(n)
y_linear = linear(n)
y_linearithmic = linearithmic(n)
y_quadratic = quadratic(n)
y_exponential = exponential(n)

# 创建图形
plt.figure(figsize=(12, 8))

# 绘制各种复杂度曲线
plt.plot(n, y_constant, label='O(1) - 常数时间', linewidth=2, color='green')
plt.plot(n, y_logarithmic, label='O(log n) - 对数时间', linewidth=2, color='blue')
plt.plot(n, y_linear, label='O(n) - 线性时间', linewidth=2, color='cyan')
plt.plot(n, y_linearithmic, label='O(n log n) - 线性对数时间', linewidth=2, color='orange')
plt.plot(n, y_quadratic, label='O(n^2) - 平方时间', linewidth=2, color='red')
plt.plot(n, y_exponential, label='O(2^n) - 指数时间', linewidth=2, color='purple')

# 添加标题和标签
plt.title('算法复杂度对比', fontsize=18)
plt.xlabel('输入规模 (n)', fontsize=14)
plt.ylabel('操作次数', fontsize=14)

# 添加网格
plt.grid(True, linestyle='--', alpha=0.7)

# 添加图例
plt.legend(fontsize=12, loc='upper left')

# 添加注释
plt.text(7, 30, 'P类问题通常具有\n多项式时间复杂度\n(如O(n), O(n^2)等)', 
         fontsize=10, bbox=dict(facecolor='yellow', alpha=0.5))
plt.text(4, 80, 'NP难问题通常需要\n指数时间解决\n(如O(2^n))', 
         fontsize=10, bbox=dict(facecolor='lightcoral', alpha=0.5))

# 设置y轴为对数刻度，使曲线更易区分
plt.yscale('log')

# 保存图片
plt.savefig('img/complexity_comparison.png', dpi=300, bbox_inches='tight')
plt.close()

print("算法复杂度对比图已保存为'img/complexity_comparison.png'") 