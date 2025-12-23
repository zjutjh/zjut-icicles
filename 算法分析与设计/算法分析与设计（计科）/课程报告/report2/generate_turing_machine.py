import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'Microsoft YaHei', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False  # 正确显示负号

# 创建图形和坐标轴
fig, ax = plt.subplots(figsize=(12, 6))

# 绘制纸带
tape_width = 0.5
tape_length = 10
tape_y = 2
cell_width = 1

# 绘制纸带背景
tape = patches.Rectangle((0, tape_y - tape_width/2), tape_length, tape_width, 
                        facecolor='lightyellow', edgecolor='black', linewidth=2)
ax.add_patch(tape)

# 绘制纸带单元格
for i in range(11):
    ax.plot([i, i], [tape_y - tape_width/2, tape_y + tape_width/2], 'k-')

# 在纸带上写入符号
symbols = ['B', '1', '0', '1', '1', '0', 'B', 'B', 'B', 'B']
for i, symbol in enumerate(symbols):
    ax.text(i + 0.5, tape_y, symbol, ha='center', va='center', fontsize=16)

# 绘制读写头
head_x = 4  # 当前读写头位置
head_width = 0.8
head_height = 1.2

# 读写头三角形
head_triangle = patches.Polygon([[head_x + 0.5, tape_y - tape_width/2 - 0.1],
                                [head_x + 0.5 - head_width/2, tape_y - tape_width/2 - head_height],
                                [head_x + 0.5 + head_width/2, tape_y - tape_width/2 - head_height]],
                               facecolor='royalblue', edgecolor='black')
ax.add_patch(head_triangle)

# 绘制状态控制器
state_x = head_x + 0.5
state_y = tape_y - tape_width/2 - head_height - 1
state_radius = 0.8

state_circle = patches.Circle((state_x, state_y), state_radius, 
                             facecolor='lightgreen', edgecolor='black')
ax.add_patch(state_circle)
ax.text(state_x, state_y, 'q₂', ha='center', va='center', fontsize=16, fontweight='bold')

# 绘制转移函数表
table_x = 1
table_y = 0.5
table_width = 6
table_height = 2

table = patches.Rectangle((table_x, table_y - table_height), table_width, table_height, 
                         facecolor='lightgray', edgecolor='black', alpha=0.8)
ax.add_patch(table)

# 添加表格标题
ax.text(table_x + table_width/2, table_y, '转移函数 δ(q, σ) = (q′, σ′, D)', 
       ha='center', va='bottom', fontsize=12, fontweight='bold')

# 添加表格内容
table_content = [
    "δ(q₁, 0) = (q₂, 1, R)",
    "δ(q₁, 1) = (q₃, 0, L)",
    "δ(q₂, 0) = (q₄, 1, R)",
    "δ(q₂, 1) = (q₂, 1, R)",
    "..."
]

for i, content in enumerate(table_content):
    ax.text(table_x + 0.2, table_y - 0.4 - i * 0.3, content, 
           ha='left', va='center', fontsize=10)

# 绘制动作箭头和标签
# 读取当前符号
read_arrow = patches.FancyArrowPatch((head_x + 0.5, tape_y - tape_width/2 - 0.2),
                                    (head_x + 0.5, tape_y - tape_width/2 - 0.6),
                                    arrowstyle='->', mutation_scale=15, color='red')
ax.add_patch(read_arrow)
ax.text(head_x + 0.8, tape_y - tape_width/2 - 0.4, '读', color='red', ha='left', va='center', fontsize=12)

# 写入新符号
write_arrow = patches.FancyArrowPatch((head_x + 0.5, tape_y - tape_width/2 - 0.8),
                                     (head_x + 0.5, tape_y - tape_width/2 - 0.6),
                                     arrowstyle='->', mutation_scale=15, color='blue')
ax.add_patch(write_arrow)
ax.text(head_x + 0.2, tape_y - tape_width/2 - 0.8, '写', color='blue', ha='right', va='center', fontsize=12)

# 移动读写头
move_arrow = patches.FancyArrowPatch((head_x + 1.2, tape_y - tape_width/2 - 0.7),
                                    (head_x + 1.8, tape_y - tape_width/2 - 0.7),
                                    arrowstyle='->', mutation_scale=15, color='green')
ax.add_patch(move_arrow)
ax.text(head_x + 1.5, tape_y - tape_width/2 - 0.5, '移动', color='green', ha='center', va='bottom', fontsize=12)

# 状态转移
state_arrow = patches.FancyArrowPatch((state_x + state_radius, state_y),
                                     (state_x + state_radius + 1, state_y),
                                     arrowstyle='->', mutation_scale=15, color='purple', connectionstyle='arc3,rad=0.3')
ax.add_patch(state_arrow)
ax.text(state_x + state_radius + 0.5, state_y + 0.3, '状态转移', color='purple', ha='center', va='bottom', fontsize=12)

# 绘制下一个状态
next_state_circle = patches.Circle((state_x + 2.5, state_y), state_radius, 
                                  facecolor='lightsalmon', edgecolor='black', alpha=0.6)
ax.add_patch(next_state_circle)
ax.text(state_x + 2.5, state_y, 'q₄', ha='center', va='center', fontsize=16, fontweight='bold')

# 设置坐标轴范围
ax.set_xlim(-0.5, 11)
ax.set_ylim(-2, 3)

# 隐藏坐标轴
ax.axis('off')

# 添加标题
ax.set_title('图灵机工作流程示意图', fontsize=18)

# 添加说明
ax.text(5.5, -1.8, 
        "图灵机在每一步操作中：1.读取当前符号 2.根据当前状态和符号查询转移函数\n"
        "3.写入新符号 4.移动读写头 5.转换到新状态",
        horizontalalignment='center',
        fontsize=10)

# 保存图片
plt.savefig('img/turing_machine_workflow.png', dpi=300, bbox_inches='tight')
plt.close()

print("图灵机工作流程示意图已保存为'img/turing_machine_workflow.png'") 