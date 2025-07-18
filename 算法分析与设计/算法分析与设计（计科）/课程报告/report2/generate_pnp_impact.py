import matplotlib.pyplot as plt
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'Microsoft YaHei', 'sans-serif']  # 增加备用字体
plt.rcParams['axes.unicode_minus'] = False  # 正确显示负号

# 创建图形
fig, ax = plt.subplots(figsize=(12, 8))

# 定义影响领域和具体影响
domains = ['算法设计', '密码学', '人工智能', '科学研究', '经济与社会']
impacts = {
    '算法设计': [
        '所有NP问题都有多项式时间算法',
        '组合优化问题变得易于解决',
        '算法设计范式的根本性改变',
        '计算机科学教育的重构'
    ],
    '密码学': [
        '现有的公钥加密系统可能被破解',
        'RSA、椭圆曲线等加密算法不再安全',
        '需要开发新的密码学理论基础',
        '量子密码学可能成为主流'
    ],
    '人工智能': [
        '机器学习算法效率大幅提升',
        '复杂规划问题变得容易解决',
        'AI创造力与推理能力的飞跃',
        '通用人工智能的实现可能加速'
    ],
    '科学研究': [
        '蛋白质折叠等生物学问题的突破',
        '新材料设计与发现的加速',
        '复杂系统模拟的精确化',
        '基础物理学理论验证的新方法'
    ],
    '经济与社会': [
        '产业结构的剧变',
        '新的商业模式涌现',
        '隐私与安全概念的重新定义',
        '可能导致数字鸿沟的扩大'
    ]
}

# 设置颜色
colors = ['#FF9999', '#66B2FF', '#99FF99', '#FFCC99', '#CC99FF']

# 绘制中心节点
center_x, center_y = 0.5, 0.5
center_radius = 0.15
center_circle = plt.Circle((center_x, center_y), center_radius, 
                          color='gold', alpha=0.7)
ax.add_patch(center_circle)
ax.text(center_x, center_y, 'P = NP', ha='center', va='center', 
       fontsize=20, fontweight='bold')

# 计算各个领域节点的位置（围绕中心的圆形布局）
n_domains = len(domains)
domain_radius = 0.35
domain_positions = []

for i in range(n_domains):
    angle = 2 * np.pi * i / n_domains
    x = center_x + domain_radius * np.cos(angle)
    y = center_y + domain_radius * np.sin(angle)
    domain_positions.append((x, y))

# 绘制领域节点和连接线
for i, (domain, pos) in enumerate(zip(domains, domain_positions)):
    # 绘制领域节点
    domain_circle = plt.Circle(pos, 0.1, color=colors[i], alpha=0.7)
    ax.add_patch(domain_circle)
    ax.text(pos[0], pos[1], domain, ha='center', va='center', 
           fontsize=12, fontweight='bold')
    
    # 绘制连接中心的线
    ax.plot([center_x, pos[0]], [center_y, pos[1]], 
           color='gray', linestyle='-', linewidth=2, alpha=0.6)
    
    # 计算并绘制具体影响
    impacts_list = impacts[domain]
    n_impacts = len(impacts_list)
    
    for j, impact in enumerate(impacts_list):
        # 计算影响节点位置
        angle_offset = (j - (n_impacts - 1) / 2) * np.pi / (3 * n_impacts)
        angle = 2 * np.pi * i / n_domains + angle_offset
        impact_radius = 0.6
        impact_x = center_x + impact_radius * np.cos(angle)
        impact_y = center_y + impact_radius * np.sin(angle)
        
        # 绘制影响节点
        impact_circle = plt.Circle((impact_x, impact_y), 0.07, 
                                  color=colors[i], alpha=0.4)
        ax.add_patch(impact_circle)
        
        # 添加影响文本（根据位置调整对齐方式）
        ha = 'left' if impact_x > center_x else 'right'
        va = 'bottom' if impact_y > center_y else 'top'
        
        # 为文本添加背景框
        ax.text(impact_x, impact_y, impact, ha=ha, va=va, 
               fontsize=9, bbox=dict(facecolor='white', alpha=0.7, 
                                    boxstyle='round,pad=0.5'))
        
        # 绘制连接领域节点和影响节点的线
        ax.plot([pos[0], impact_x], [pos[1], impact_y], 
               color=colors[i], linestyle='-', linewidth=1.5, alpha=0.4)

# 设置标题
ax.set_title('P=NP对各领域的潜在影响', fontsize=18)

# 设置坐标轴范围
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_aspect('equal')

# 关闭坐标轴
ax.axis('off')

# 保存图片
plt.savefig('./img/pnp_impact.png', dpi=300, bbox_inches='tight')
plt.close()

print("P=NP影响示意图已生成到 ./img/pnp_impact.png") 