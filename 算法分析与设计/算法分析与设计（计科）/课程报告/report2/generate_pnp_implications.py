import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'Microsoft YaHei', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False  # 正确显示负号

# 创建图形
fig, ax = plt.subplots(figsize=(12, 10))

# 设置背景颜色
ax.set_facecolor('#f8f8f8')

# 创建中心节点
center_x, center_y = 0.5, 0.5
center_radius = 0.1
center = plt.Circle((center_x, center_y), center_radius, 
                   color='gold', alpha=0.8, ec='black')
ax.add_patch(center)
ax.text(center_x, center_y, 'P=NP', ha='center', va='center', 
        fontsize=16, fontweight='bold')

# 定义影响领域
domains = [
    {'name': '密码学', 'color': '#FF9999', 'angle': 0},
    {'name': '算法设计', 'color': '#66B2FF', 'angle': 72},
    {'name': '人工智能', 'color': '#99FF99', 'angle': 144},
    {'name': '科学研究', 'color': '#FFCC99', 'angle': 216},
    {'name': '社会经济', 'color': '#CC99FF', 'angle': 288}
]

# 计算领域节点的位置
domain_radius = 0.25
for domain in domains:
    angle_rad = np.radians(domain['angle'])
    x = center_x + domain_radius * np.cos(angle_rad)
    y = center_y + domain_radius * np.sin(angle_rad)
    domain['x'] = x
    domain['y'] = y

# 绘制领域节点和连接线
for domain in domains:
    # 绘制领域节点
    domain_circle = plt.Circle((domain['x'], domain['y']), 0.08, 
                              color=domain['color'], alpha=0.8, ec='black')
    ax.add_patch(domain_circle)
    ax.text(domain['x'], domain['y'], domain['name'], ha='center', va='center', 
           fontsize=11, fontweight='bold')
    
    # 绘制连接中心的线
    ax.plot([center_x, domain['x']], [center_y, domain['y']], 
           color='gray', linestyle='-', linewidth=2, alpha=0.6)

# 添加具体影响
impacts = {
    '密码学': [
        {'text': '现有加密系统可能被破解', 'pos': 0.2},
        {'text': '需要全新的密码学理论', 'pos': 0.4},
        {'text': '量子密码学成为主流', 'pos': 0.6}
    ],
    '算法设计': [
        {'text': 'NP完全问题有多项式解法', 'pos': 0.2},
        {'text': '组合优化问题变得容易', 'pos': 0.4},
        {'text': '算法设计范式根本改变', 'pos': 0.6}
    ],
    '人工智能': [
        {'text': '机器学习效率大幅提升', 'pos': 0.2},
        {'text': '复杂规划问题易于解决', 'pos': 0.4},
        {'text': 'AI创造力与推理能力飞跃', 'pos': 0.6}
    ],
    '科学研究': [
        {'text': '蛋白质折叠等问题突破', 'pos': 0.2},
        {'text': '新材料设计加速', 'pos': 0.4},
        {'text': '复杂系统模拟精确化', 'pos': 0.6}
    ],
    '社会经济': [
        {'text': '产业结构剧变', 'pos': 0.2},
        {'text': '新商业模式涌现', 'pos': 0.4},
        {'text': '隐私与安全概念重定义', 'pos': 0.6}
    ]
}

# 绘制具体影响
for domain in domains:
    domain_impacts = impacts[domain['name']]
    angle_rad = np.radians(domain['angle'])
    
    for impact in domain_impacts:
        impact_radius = 0.25 + impact['pos']
        impact_x = center_x + impact_radius * np.cos(angle_rad)
        impact_y = center_y + impact_radius * np.sin(angle_rad)
        
        # 绘制影响节点
        impact_circle = plt.Circle((impact_x, impact_y), 0.05, 
                                  color=domain['color'], alpha=0.6, ec='black')
        ax.add_patch(impact_circle)
        
        # 计算文本角度，使其更易读
        text_angle = domain['angle']
        if 90 < text_angle < 270:
            text_angle += 180  # 翻转文本方向
        
        # 添加影响文本
        ax.text(impact_x, impact_y + 0.07, impact['text'], 
               ha='center', va='center', fontsize=9,
               rotation=0,  # 不旋转文本
               bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.3'))
        
        # 绘制连接线
        ax.plot([domain['x'], impact_x], [domain['y'], impact_y], 
               color=domain['color'], linestyle='-', linewidth=1.5, alpha=0.6)

# 添加P≠NP的情况
pneq_x, pneq_y = 0.5, 0.9
# 修复：移除了boxstyle参数，使用FancyBboxPatch替代
pneq_rect = patches.Rectangle((pneq_x - 0.25, pneq_y - 0.05), 0.5, 0.1, 
                             facecolor='lightgray', edgecolor='black',
                             alpha=0.8, linewidth=1, zorder=2)
ax.add_patch(pneq_rect)
ax.text(pneq_x, pneq_y, 'P≠NP的影响', ha='center', va='center', 
        fontsize=14, fontweight='bold')

# 添加P≠NP的影响
pneq_impacts = [
    {'text': '计算复杂性理论深化', 'x': 0.2, 'y': 0.8},
    {'text': '近似算法更受重视', 'x': 0.4, 'y': 0.8},
    {'text': '启发式方法继续发展', 'x': 0.6, 'y': 0.8},
    {'text': '量子计算成为希望', 'x': 0.8, 'y': 0.8}
]

for impact in pneq_impacts:
    # 绘制影响节点
    impact_circle = plt.Circle((impact['x'], impact['y']), 0.03, 
                              color='darkgray', alpha=0.8, ec='black')
    ax.add_patch(impact_circle)
    
    # 添加影响文本
    ax.text(impact['x'], impact['y'] - 0.05, impact['text'], 
           ha='center', va='center', fontsize=8,
           bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.2'))
    
    # 绘制连接线
    ax.plot([pneq_x, impact['x']], [pneq_y - 0.05, impact['y'] + 0.03], 
           color='darkgray', linestyle='-', linewidth=1, alpha=0.6)

# 添加标题
ax.set_title('P=NP问题的潜在影响与应用', fontsize=18)

# 添加说明
ax.text(0.5, 0.05, 
        "注：P=NP问题的解决，无论结果如何，都将对计算机科学和人类社会产生深远影响。\n"
        "如果P=NP成立，许多目前认为难解的问题将变得容易；如果P≠NP成立，则确认了计算的基本限制。",
        horizontalalignment='center',
        fontsize=10,
        bbox=dict(facecolor='white', alpha=0.7))

# 设置坐标轴范围
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)

# 隐藏坐标轴
ax.axis('off')

# 保存图片
plt.savefig('img/pnp_implications.png', dpi=300, bbox_inches='tight')
plt.close()

print("P=NP问题的潜在影响与应用图已保存为'img/pnp_implications.png'") 