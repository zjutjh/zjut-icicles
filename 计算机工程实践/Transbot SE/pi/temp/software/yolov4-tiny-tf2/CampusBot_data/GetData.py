import os
import math
import cv2 as cv
import numpy as np
import imgaug.augmenters as iaa
from collections import Counter

# --- 配置区 ---
# 生成的图片总数
IMG_TOTAL = 150
# 图片尺寸
IMG_SIZE = 416

# 类别映射表：将 image/ 目录下的图片编号 (0.jpg, 1.jpg...) 映射到最终的类别ID
# 类别ID从0开始
IMGID_TO_CLASSID = {
    0: 0,  # fire
    1: 1,  # turn_round
    2: 2,  # turn_right
    3: 3,  # turn_left
    4: 4,  # construction
    5: 5,  # stop
}

# 类别名称列表：顺序必须和上面的类别ID对应！
CLASS_NAMES = [
    "fire",
    "turn_round",
    "turn_right",
    "turn_left",
    "construction",
    "stop",
]
# --- 配置区结束 ---


def rotate_about_center(src, angle, scale=1.):
    h, w = src.shape[:2]
    # angle in radians
    rangle = np.deg2rad(angle)
    # now calculate new image width and height
    nw = (abs(np.sin(rangle) * h) + abs(np.cos(rangle) * w)) * scale
    nh = (abs(np.cos(rangle) * h) + abs(np.sin(rangle) * w)) * scale
    # ask OpenCV for the rotation matrix
    rot_mat = cv.getRotationMatrix2D((nw * 0.5, nh * 0.5), angle, scale)
    # calculate the move from the old center to the new center combined with the rotation
    rot_move = np.dot(rot_mat, np.array([(nw - w) * 0.5, (nh - h) * 0.5, 0]))
    # the move only affects the translation, so update the translation part of the transform
    rot_mat[0, 2] += rot_move[0]
    rot_mat[1, 2] += rot_move[1]
    # Applies an affine transformation to an image.
    tempImg = cv.warpAffine(src, rot_mat, (int(math.ceil(nw)), int(math.ceil(nh))),
                            flags=cv.INTER_LANCZOS4,
                            borderValue=(0, 255, 0))
    seedx = np.linspace(0.4, 0.8, 20)
    seedy = np.linspace(0.4, 0.8, 20)
    indexx = np.random.randint(0, len(seedx))
    indexy = np.random.randint(0, len(seedy))
    res = cv.resize(tempImg, (int(tempImg.shape[1] * seedx[indexx]),
                              int(tempImg.shape[0] * seedy[indexy])),
                    interpolation=cv.INTER_CUBIC)  # dsize=（2*width,2*height）
    return res


def roate_zoom_img(img):
    angle = np.linspace(-30, 30, 20)
    scale = np.linspace(0.8, 1.2, 20)
    index = np.random.randint(0, len(angle))
    # new_img = rotate_about_center(img, 0, scale[index])
    new_img = rotate_about_center(img, angle[index], scale[index])
    return new_img


def img_random_brightness(img):
    brightness = iaa.Multiply((0.7, 1.3))
    # print(brightness.augment)
    image = brightness.augment_image(img)
    return image


def random_augment(img):
    image = img_random_brightness(img)
    image = roate_zoom_img(image)
    return image


def checkOverlap(rectangles, b):
    # 快速检查：如果没有矩形，则不会重叠
    if not rectangles:
        return False
        
    # 从b提取坐标
    b_x1, b_y1, b_x2, b_y2 = b
    b_width = b_x2 - b_x1
    b_height = b_y2 - b_y1
    
    # 使用向量操作一次性检查所有矩形
    if len(rectangles) > 10:  # 当矩形较多时使用更高效的算法
        # 将矩形转换为numpy数组，加速批量计算
        rects = np.array(rectangles)
        
        # 提取坐标
        a_x1 = rects[:, 0]
        a_y1 = rects[:, 1]
        a_x2 = rects[:, 2]
        a_y2 = rects[:, 3]
        
        # 计算宽度和高度
        a_width = a_x2 - a_x1
        a_height = a_y2 - a_y1
        
        # 计算重叠条件
        widthmin = np.minimum.outer(np.append(a_x1, b_x1), np.append(a_x2, b_x2))
        widthmax = np.maximum.outer(np.append(a_x1, b_x1), np.append(a_x2, b_x2))
        
        heightmin = np.minimum.outer(np.append(a_y1, b_y1), np.append(a_y2, b_y2))
        heightmax = np.maximum.outer(np.append(a_y1, b_y1), np.append(a_y2, b_y2))
        
        # 检查是否有重叠
        width_overlap = (a_width + b_width) >= (widthmax - widthmin)
        height_overlap = (a_height + b_height) >= (heightmax - heightmin)
        
        # 如果任何一个矩形满足重叠条件，返回True
        if np.any(width_overlap & height_overlap):
            return True
    else:
        # 对较少的矩形使用原始循环方法
        for a in rectangles:
            # 两个矩形不重叠的条件：
            # 1. 一个在另一个的右侧
            # 2. 一个在另一个的左侧
            # 3. 一个在另一个的上方
            # 4. 一个在另一个的下方
            if not (a[2] < b[0] or a[0] > b[2] or a[3] < b[1] or a[1] > b[3]):
                return True
    
    return False


# 随机位置生成图片
def transparentOverlay(bg_path, class_counts):
    bgImg = cv.imread(bg_path, -1)
    target = cv.resize(bgImg, (IMG_SIZE, IMG_SIZE))
    
    annotations = []
    rectangles_for_overlap_check = []

    # 减少随机对象数量，加快生成速度
    num_objects_to_add = np.random.randint(5, 15)
    
    # 预加载所有可用图片，避免重复读取
    script_dir = os.path.dirname(__file__)
    available_images = {}
    
    # 创建类别到imgid的映射，加速查找
    class_to_imgids = {}
    for imgid, classid in IMGID_TO_CLASSID.items():
        if classid not in class_to_imgids:
            class_to_imgids[classid] = []
        class_to_imgids[classid].append(imgid)
    
    # 为每个类别预先检查并加载一次图像
    for classid in class_to_imgids:
        for imgid in class_to_imgids[classid]:
            img_path = os.path.join(script_dir, 'image', f'{imgid}.jpg')
            if not os.path.exists(img_path):
                img_path = os.path.join(script_dir, 'image', f'{imgid}.png')
                if not os.path.exists(img_path):
                    continue
                
            img = cv.imread(img_path)
            if img is not None:
                available_images[imgid] = img
    
    # 提前计算可能的放置位置
    valid_areas = []
    grid_size = 20  # 网格大小，越小精度越高但速度越慢
    for x in range(0, IMG_SIZE, grid_size):
        for y in range(0, IMG_SIZE, grid_size):
            valid_areas.append((x, y))
    np.random.shuffle(valid_areas)
    
    success_count = 0
    max_tries = min(num_objects_to_add * 10, 100)  # 减少最大尝试次数
    current_tries = 0

    while success_count < num_objects_to_add and current_tries < max_tries:
        current_tries += 1

        # 高效地选择类别，偏向数量少的类别
        class_weights = {}
        total = sum(class_counts.values()) + 1  # 避免除零
        for c in class_counts:
            # 反向权重：数量越少权重越高
            class_weights[c] = (total - class_counts[c]) / total
            
        # 归一化权重
        weight_sum = sum(class_weights.values())
        class_probs = [class_weights[c]/weight_sum for c in sorted(class_weights.keys())]
        classid_to_add = np.random.choice(sorted(class_weights.keys()), p=class_probs)
        
        # 获取可用的图片ID
        imgids = class_to_imgids.get(classid_to_add, [])
        if not imgids or all(i not in available_images for i in imgids):
            continue
        
        # 过滤出已加载的图片
        valid_imgids = [i for i in imgids if i in available_images]
        if not valid_imgids:
            continue
            
        imgid = np.random.choice(valid_imgids)
        readimg = available_images[imgid]
        
        # 图像增强
        overlay = random_augment(readimg)
        h, w, _ = overlay.shape

        if h >= IMG_SIZE or w >= IMG_SIZE:
            continue

        # 尝试放置图像
        placed = False
        for x_base, y_base in valid_areas:
            if x_base + w >= IMG_SIZE or y_base + h >= IMG_SIZE:
                continue
                
            # 添加一些随机偏移
            offset_x = np.random.randint(0, min(grid_size, IMG_SIZE - x_base - w))
            offset_y = np.random.randint(0, min(grid_size, IMG_SIZE - y_base - h))
            
            x_coord = x_base + offset_x
            y_coord = y_base + offset_y
            
            bbox = (x_coord, y_coord, x_coord + w, y_coord + h)
            
            if not checkOverlap(rectangles_for_overlap_check, bbox):
                placed = True
                rectangles_for_overlap_check.append(bbox)
                annotations.append(bbox + (imgid,))
                
                # 使用掩码进行高效粘贴
                hsv = cv.cvtColor(overlay, cv.COLOR_BGR2HSV)
                # 扩大HSV范围以适应更多背景颜色
                mask = cv.inRange(hsv, (30, 30, 30), (90, 255, 255))
                mask_inv = cv.bitwise_not(mask)
                
                # 为提高速度，使用NumPy操作而不是逐像素循环
                roi = target[y_coord:y_coord+h, x_coord:x_coord+w]
                img_bg = cv.bitwise_and(roi, roi, mask=mask)
                img_fg = cv.bitwise_and(overlay, overlay, mask=mask_inv)
                dst = cv.add(img_bg, img_fg)
                target[y_coord:y_coord+h, x_coord:x_coord+w] = dst
                
                class_counts[classid_to_add] += 1
                success_count += 1
                break
                
        if not placed:
            # 如果无法通过网格系统找到位置，随机尝试
            for _ in range(3):  # 限制随机尝试次数
                y_coord = np.random.randint(0, IMG_SIZE - h)
                x_coord = np.random.randint(0, IMG_SIZE - w)
                
                bbox = (x_coord, y_coord, x_coord + w, y_coord + h)
                
                if not checkOverlap(rectangles_for_overlap_check, bbox):
                    rectangles_for_overlap_check.append(bbox)
                    annotations.append(bbox + (imgid,))
                    
                    # 使用掩码进行高效粘贴
                    hsv = cv.cvtColor(overlay, cv.COLOR_BGR2HSV)
                    # 扩大HSV范围以适应更多背景颜色
                    mask = cv.inRange(hsv, (30, 30, 30), (90, 255, 255))
                    mask_inv = cv.bitwise_not(mask)
                    
                    roi = target[y_coord:y_coord+h, x_coord:x_coord+w]
                    img_bg = cv.bitwise_and(roi, roi, mask=mask)
                    img_fg = cv.bitwise_and(overlay, overlay, mask=mask_inv)
                    dst = cv.add(img_bg, img_fg)
                    target[y_coord:y_coord+h, x_coord:x_coord+w] = dst
                    
                    class_counts[classid_to_add] += 1
                    success_count += 1
                    placed = True
                    break
    
    print(f"放置了 {success_count}/{num_objects_to_add} 个对象")
    return target, annotations, class_counts


def generate_ready_to_train_dataset(total_images):
    import time
    start_time = time.time()
    
    base_dir = os.path.dirname(__file__)
    project_root = os.path.dirname(base_dir)
    texture_dir = os.path.join(base_dir, 'texture')
    jpeg_dir = os.path.join(base_dir, 'JPEGImages')
    model_data_dir = os.path.join(project_root, 'model_data')

    os.makedirs(jpeg_dir, exist_ok=True)
    os.makedirs(model_data_dir, exist_ok=True)

    # 预加载所有纹理图片
    texture_list = [f for f in os.listdir(texture_dir) if f.endswith(('.jpg', '.png'))]
    if not texture_list:
        print(f"错误: Texture 目录 '{texture_dir}' 为空或没有图片!")
        return
        
    # 预先准备好纹理图片路径
    texture_paths = [os.path.join(texture_dir, f) for f in texture_list]
    
    # 多线程处理，用于加快图像生成
    from concurrent.futures import ThreadPoolExecutor
    import threading
    
    # 线程安全的计数器和列表
    lock = threading.Lock()
    class_counts = Counter({i: 0 for i in range(len(CLASS_NAMES))})
    image_path_list = []
    
    # 进度跟踪变量
    completed = 0
    print("--- 开始生成可直接训练的数据集 ---")
    
    def process_image(i):
        nonlocal completed
        
        # 随机选择背景图片
        bg_path = np.random.choice(texture_paths)
        
        # 创建本地副本以避免多线程冲突
        local_counts = Counter({i: 0 for i in range(len(CLASS_NAMES))})
        for k, v in class_counts.items():
            local_counts[k] = v
            
        # 生成图像和标注
        generated_image, annotations, updated_counts = transparentOverlay(bg_path, local_counts)
        
        # 保存图像
        img_filename = f"{i}.jpg"
        img_save_path = os.path.join(jpeg_dir, img_filename)
        cv.imwrite(img_save_path, generated_image)

        # 准备YOLO格式的标注
        img_h, img_w, _ = generated_image.shape
        yolo_annotation_lines = []
        
        for (x1, y1, x2, y2, imgid) in annotations:
            classid = IMGID_TO_CLASSID.get(imgid, 0)
            
            box_w = float(x2 - x1)
            box_h = float(y2 - y1)
            x_center = float(x1) + box_w / 2
            y_center = float(y1) + box_h / 2
            
            x_center_norm = x_center / img_w
            y_center_norm = y_center / img_h
            box_w_norm = box_w / img_w
            box_h_norm = box_h / img_h
            
            yolo_line = f"{classid} {x_center_norm:.6f} {y_center_norm:.6f} {box_w_norm:.6f} {box_h_norm:.6f}"
            yolo_annotation_lines.append(yolo_line)

        # 保存标注文件
        label_filename = f"{i}.txt"
        label_save_path = os.path.join(jpeg_dir, label_filename)
        with open(label_save_path, 'w', encoding='utf-8') as f:
            f.write("\n".join(yolo_annotation_lines))

        # 更新图片路径列表和类别计数
        data_dir_name = os.path.basename(base_dir)
        relative_img_path = os.path.join(data_dir_name, 'JPEGImages', img_filename).replace('\\', '/')
        
        with lock:
            image_path_list.append(relative_img_path)
            for k, v in updated_counts.items():
                class_counts[k] = max(class_counts[k], v)
            
            # 更新进度
            completed += 1
            if completed % 10 == 0 or completed == total_images:
                elapsed = time.time() - start_time
                images_per_sec = completed / elapsed if elapsed > 0 else 0
                print(f"已生成 {completed}/{total_images} 张图片和标注... {images_per_sec:.2f} 图/秒")

    # 使用线程池并行生成图像
    with ThreadPoolExecutor(max_workers=min(8, os.cpu_count() or 4)) as executor:
        executor.map(process_image, range(total_images))
    
    # 生成完成后，保存训练列表和类别文件
    train_txt_path = os.path.join(base_dir, "train.txt")
    with open(train_txt_path, 'w', encoding='utf-8') as f:
        f.write("\n".join(image_path_list))
    
    classes_txt_path = os.path.join(model_data_dir, 'CampusBot.txt')
    with open(classes_txt_path, 'w', encoding='utf-8') as f:
        f.write("\n".join(CLASS_NAMES))

    # 计算总耗时和性能
    total_time = time.time() - start_time
    imgs_per_sec = total_images / total_time if total_time > 0 else 0
    
    print(f"\n--- 数据集生成完毕! 总耗时: {total_time:.2f}秒 (平均 {imgs_per_sec:.2f} 图/秒) ---")
    print(f"图片保存在: {jpeg_dir}")
    print(f"YOLO标注保存在: {jpeg_dir}")
    print(f"训练列表文件: {train_txt_path}")
    print(f"类别名称文件: {classes_txt_path}")
    print("\n现在你可以使用 'train.txt' 和 'CampusBot.txt' 文件路径去训练你的模型了。")

    # 额外分析：检查类别均衡性
    print("\n各类别最终统计数量：")
    for class_id, count in sorted(class_counts.items()):
        class_name = CLASS_NAMES[class_id] if class_id < len(CLASS_NAMES) else f"未知类别{class_id}"
        print(f"  - {class_name}: {count} 个实例")


if __name__ == '__main__':
    generate_ready_to_train_dataset(IMG_TOTAL)
