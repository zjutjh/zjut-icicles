import numpy as np
import os
import random

def cas_iou(box, cluster):
    x = np.minimum(cluster[:, 0], box[0])
    y = np.minimum(cluster[:, 1], box[1])

    intersection = x * y
    area1 = box[0] * box[1]

    area2 = cluster[:, 0] * cluster[:, 1]
    iou = intersection / (area1 + area2 - intersection)

    return iou

def avg_iou(box, cluster):
    return np.mean([np.max(cas_iou(box[i], cluster)) for i in range(box.shape[0])])

def kmeans(box, k):
    row = box.shape[0]
    distance = np.empty((row, k))
    last_clu = np.zeros((row,))
    np.random.seed()
    cluster = box[np.random.choice(row, k, replace=False)]

    while True:
        for i in range(row):
            distance[i] = 1 - cas_iou(box[i], cluster)
        
        near = np.argmin(distance, axis=1)

        if (last_clu == near).all():
            break
        
        for j in range(k):
            cluster[j] = np.median(box[near == j], axis=0)

        last_clu = near

    return cluster

def load_data_from_yolo(train_list_path):
    """
    从YOLO格式的数据集加载标注框的宽高。
    :param train_list_path: 指向 train.txt 的路径。
    :return: 一个Numpy数组，包含所有框的 [归一化宽度, 归一化高度]。
    """
    data = []
    # 脚本在 utils/ 目录, 项目根目录是上两级
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    try:
        with open(train_list_path, 'r') as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"错误: 找不到训练列表文件 '{train_list_path}'")
        print("请先运行 CampusBot_data/GetData.py 生成数据集。")
        return np.array(data)

    print(f"开始从 {len(lines)} 张图片中加载标注数据...")
    for line in lines:
        line = line.strip()
        if not line:
            continue
        
        # line 是 'CampusBot_data/JPEGImages/0.jpg'
        # 对应的标注文件是 'CampusBot_data/JPEGImages/0.txt'
        label_path = os.path.splitext(line)[0] + '.txt'
        full_label_path = os.path.join(project_root, label_path)

        if not os.path.exists(full_label_path):
            print(f"警告: 找不到对应的标注文件: {full_label_path}")
            continue

        with open(full_label_path, 'r') as label_f:
            for label_line in label_f:
                parts = label_line.strip().split()
                if len(parts) == 5:
                    # YOLO format: class_id x_center y_center width height
                    width = float(parts[3])
                    height = float(parts[4])
                    data.append([width, height])
                
    return np.array(data)

if __name__ == '__main__':
    # --- 配置区 ---
    # 输入图片尺寸
    SIZE = 416
    # 希望生成的 anchor 数量
    ANCHORS_NUM = 6
    # 数据集训练列表文件路径
    # 注意：我们是在 utils/ 目录运行，所以要用 '..' 来返回上一级目录
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    TRAIN_LIST_PATH = os.path.join(project_root, 'CampusBot_data', 'train.txt')
    # Anchor 文件保存路径
    ANCHOR_SAVE_PATH = os.path.join(project_root, 'model_data', 'campusbot_anchors.txt')
    # --- 配置区结束 ---

    print("--- 开始为您的数据集计算最优YOLO Anchors ---")
    
    # 1. 加载数据
    data = load_data_from_yolo(TRAIN_LIST_PATH)
    
    if data.size == 0:
        print("未能加载任何标注数据，程序退出。请检查路径和文件内容。")
    else:
        print(f"成功加载了 {data.shape[0]} 个标注框。")
        # 2. 使用K-means聚类
        print("正在进行K-Means聚类计算...")
        out = kmeans(data, ANCHORS_NUM)
        # 3. 排序和评估
        out = out[np.argsort(out[:, 0])]
        print('聚类完成！平均IoU (acc): {:.2f}%'.format(avg_iou(data, out) * 100))
        print("计算出的Anchor尺寸 (乘以图像尺寸后):")
        print(out * SIZE)
        
        # 4. 保存文件
        final_anchors = out * SIZE
        f = open(ANCHOR_SAVE_PATH, 'w')
        row = np.shape(final_anchors)[0]
        for i in range(row):
            if i == 0:
                x_y = "%d,%d" % (final_anchors[i][0], final_anchors[i][1])
            else:
                x_y = ", %d,%d" % (final_anchors[i][0], final_anchors[i][1])
            f.write(x_y)
        f.close()
        print(f"\n成功! 新的 yolo_anchors.txt 已保存到: {ANCHOR_SAVE_PATH}")