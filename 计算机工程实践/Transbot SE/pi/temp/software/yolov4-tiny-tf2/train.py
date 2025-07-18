import numpy as np
import tensorflow as tf
import tensorflow.keras.backend as K
from tensorflow.keras.layers import Input, Lambda
from tensorflow.keras.models import Model
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.callbacks import TensorBoard, ReduceLROnPlateau, EarlyStopping, Callback
from yolo_nets.yolo4_tiny import yolo_body
from yolo_nets.loss import yolo_loss
from utils.utils import get_random_data, get_random_data_with_Mosaic, rand, WarmUpCosineDecayScheduler, ModelCheckpoint
import cv2
import os
from PIL import Image, ImageDraw, ImageFont
import colorsys

# 保持 TensorFlow 2.x 默认的 eager execution，便于 tf.data.Dataset 迭代
# 如果后续需要关闭 eager，可酌情调整

#---------------------------------------------------#
#   获得类和先验框
#---------------------------------------------------#
def get_classes(classes_path):
    '''loads the classes'''
    with open(classes_path) as f:
        class_names = f.readlines()
    class_names = [c.strip() for c in class_names]
    return class_names

def get_anchors(anchors_path):
    '''loads the anchors from a file'''
    with open(anchors_path) as f:
        anchors = f.readline()
    anchors = [float(x) for x in anchors.split(',')]
    return np.array(anchors).reshape(-1, 2)

#---------------------------------------------------#
#   训练数据生成器
#---------------------------------------------------#
def data_generator(annotation_lines, batch_size, input_shape, anchors, num_classes, mosaic=False):
    '''data generator for fit_generator'''
    n = len(annotation_lines)
    i = 0
    flag = True
    while True:
        image_data = []
        box_data = []
        for b in range(batch_size):
            if i==0:
                np.random.shuffle(annotation_lines)
            if mosaic:
                if flag and (i+4) < n:
                    image, box = get_random_data_with_Mosaic(annotation_lines[i:i+4], input_shape)
                    i = (i+4) % n
                else:
                    image, box = get_random_data(annotation_lines[i], input_shape)
                    i = (i+1) % n
                flag = bool(1-flag)
            else:
                image, box = get_random_data(annotation_lines[i], input_shape)
                i = (i+1) % n
            image_data.append(image)
            box_data.append(box)
        image_data = np.array(image_data)
        box_data = np.array(box_data)
        y_true = preprocess_true_boxes(box_data, input_shape, anchors, num_classes)
        # 将其修改为返回一个元组 (inputs, targets)
        yield (image_data, y_true[0], y_true[1]), np.zeros(batch_size)

def data_generator_wrapper(annotation_lines, batch_size, input_shape, anchors, num_classes, mosaic=False):
    n = len(annotation_lines)
    if n==0 or batch_size<=0: return None
    return data_generator(annotation_lines, batch_size, input_shape, anchors, num_classes, mosaic)

#---------------------------------------------------#
#   读入xml文件，并输出y_true
#---------------------------------------------------#
def preprocess_true_boxes(true_boxes, input_shape, anchors, num_classes):
    assert (true_boxes[..., 4]<num_classes).all(), 'class id must be less than num_classes'
    # 一共有两个特征层
    num_layers = len(anchors)//3
    # 先验框
    anchor_mask = [[3,4,5], [1,2,3]]

    true_boxes = np.array(true_boxes, dtype='float32')
    input_shape = np.array(input_shape, dtype='int32') # 416,416
    # 读出xy轴，读出长宽
    # 中心点(m,n,2)
    boxes_xy = (true_boxes[..., 0:2] + true_boxes[..., 2:4]) // 2
    boxes_wh = true_boxes[..., 2:4] - true_boxes[..., 0:2]
    # 计算比例
    true_boxes[..., 0:2] = boxes_xy/input_shape[:]
    true_boxes[..., 2:4] = boxes_wh/input_shape[:]

    # m张图
    m = true_boxes.shape[0]
    # 得到网格的shape为13,13;26,26;
    grid_shapes = [input_shape//{0:32, 1:16}[l] for l in range(num_layers)]
    # y_true的格式为(m,13,13,3,85)(m,26,26,3,85)
    y_true = [np.zeros((m,grid_shapes[l][0],grid_shapes[l][1],len(anchor_mask[l]),5+num_classes),
        dtype='float32') for l in range(num_layers)]
    # [1,9,2]
    anchors = np.expand_dims(anchors, 0)
    anchor_maxes = anchors / 2.
    anchor_mins = -anchor_maxes
    # 长宽要大于0才有效
    valid_mask = boxes_wh[..., 0]>0

    for b in range(m):
        # 对每一张图进行处理
        wh = boxes_wh[b, valid_mask[b]]
        if len(wh)==0: continue
        # [n,1,2]
        wh = np.expand_dims(wh, -2)
        box_maxes = wh / 2.
        box_mins = -box_maxes

        # 计算真实框和哪个先验框最契合
        intersect_mins = np.maximum(box_mins, anchor_mins)
        intersect_maxes = np.minimum(box_maxes, anchor_maxes)
        intersect_wh = np.maximum(intersect_maxes - intersect_mins, 0.)
        intersect_area = intersect_wh[..., 0] * intersect_wh[..., 1]
        box_area = wh[..., 0] * wh[..., 1]
        anchor_area = anchors[..., 0] * anchors[..., 1]
        iou = intersect_area / (box_area + anchor_area - intersect_area)
        # 维度是(n)
        best_anchor = np.argmax(iou, axis=-1)

        for t, n in enumerate(best_anchor):
            for l in range(num_layers):
                if n in anchor_mask[l]:
                    # floor用于向下取整
                    i = np.floor(true_boxes[b,t,0]*grid_shapes[l][1]).astype('int32')
                    j = np.floor(true_boxes[b,t,1]*grid_shapes[l][0]).astype('int32')
                    # 找到真实框在特征层l中第b副图像对应的位置
                    k = anchor_mask[l].index(n)
                    c = true_boxes[b,t, 4].astype('int32')
                    y_true[l][b, j, i, k, 0:4] = true_boxes[b,t, 0:4]
                    y_true[l][b, j, i, k, 4] = 1
                    y_true[l][b, j, i, k, 5+c] = 1

    return y_true

#---------------------------------------------------#
#   验证集可视化回调
#---------------------------------------------------#
class VisualizeValidationCallback(Callback):
    def __init__(self, validation_data, anchors, class_names, input_shape, log_dir='logs/validation_images'):
        super(VisualizeValidationCallback, self).__init__()
        self.validation_data = validation_data
        self.anchors = anchors
        self.class_names = class_names
        self.input_shape = input_shape
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        
        # 为每个类别生成不同的颜色
        hsv_tuples = [(x / len(class_names), 1., 1.) for x in range(len(class_names))]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), self.colors))
        
    def on_epoch_end(self, epoch, logs=None):
        # 每个epoch结束时保存一部分验证图像
        print("\n正在生成验证图像...")
        
        # 创建当前epoch的文件夹
        epoch_dir = os.path.join(self.log_dir, f'epoch_{epoch+1}')
        os.makedirs(epoch_dir, exist_ok=True)
        
        # 直接使用训练数据文件作为验证图像源
        with open('CampusBot_data/train.txt') as f:
            lines = f.readlines()
        
        # 随机选择5张图片作为验证图像
        if len(lines) > 0:
            np.random.shuffle(lines)
            num_val = min(5, len(lines))
            
            for i in range(num_val):
                image_path = lines[i].strip()
                
                # 获取对应的标注文件路径
                txt_path = image_path.replace('.jpg', '.txt')
                
                try:
                    # 加载图像
                    image = Image.open(image_path)
                    image_width, image_height = image.size
                    
                    # 准备图像绘制
                    draw = ImageDraw.Draw(image)
                    
                    # 尝试加载字体，如果失败则使用默认字体
                    try:
                        font = ImageFont.truetype("font/Block_Simplified.TTF", 15)
                    except IOError:
                        font = ImageFont.load_default()
                    
                    # 读取标注文件
                    boxes = []
                    if os.path.exists(txt_path):
                        with open(txt_path, 'r') as f:
                            for line in f.readlines():
                                data = line.strip().split()
                                if len(data) == 5:
                                    # YOLO格式: class_id center_x center_y width height
                                    cls_id = int(data[0])
                                    center_x = float(data[1]) * image_width
                                    center_y = float(data[2]) * image_height
                                    width = float(data[3]) * image_width
                                    height = float(data[4]) * image_height
                                    
                                    # 转换为(x_min, y_min, x_max, y_max)
                                    x_min = max(0, int(center_x - width/2))
                                    y_min = max(0, int(center_y - height/2))
                                    x_max = min(image_width, int(center_x + width/2))
                                    y_max = min(image_height, int(center_y + height/2))
                                    
                                    boxes.append([x_min, y_min, x_max, y_max, cls_id])
                    
                    # 绘制每个边界框
                    print(f'处理图像 {image_path}，找到 {len(boxes)} 个标注框')
                    for box in boxes:
                        x_min, y_min, x_max, y_max, cls_id = box
                        
                        # 确保类别ID在范围内
                        if cls_id >= len(self.colors):
                            cls_id = 0
                        
                        # 绘制矩形
                        thickness = 3
                        draw.rectangle([x_min, y_min, x_max, y_max], outline=self.colors[int(cls_id)], width=thickness)
                        
                        # 绘制类别标签
                        if cls_id < len(self.class_names):
                            class_name = self.class_names[int(cls_id)]
                            text = f'{class_name}'
                            # 使用 getsize 代替废弃的 textsize 方法
                            try:
                                text_size = font.getsize(text)
                            except:
                                # 最后的备选方案
                                text_size = (len(text) * 8, 15)
                            
                            draw.rectangle([x_min, y_min, x_min + text_size[0] + 5, y_min + text_size[1] + 5], 
                                        fill=self.colors[int(cls_id)])
                            draw.text((x_min, y_min), text, fill=(255, 255, 255), font=font)
                    
                    # 保存图像
                    save_path = os.path.join(epoch_dir, f'val_image_{i}.jpg')
                    image.save(save_path)
                    print(f'已保存验证图像到 {save_path}')
                    
                except Exception as e:
                    print(f'处理验证图像时出错: {str(e)}')
                    continue
        
        print(f"验证图像已保存到 {epoch_dir}")

if __name__ == "__main__":
    # 标签的位置
    annotation_path = 'CampusBot_data/train.txt'
    # 获取classes和anchor的位置
    classes_path = 'model_data/CampusBot.txt'
    anchors_path = 'model_data/CampusBot_anchors.txt'
    # 预训练模型的位置
    weights_path = 'model_data/yolov4_tiny_weights_coco.h5'
    # 获得classes和anchor
    class_names = get_classes(classes_path)
    anchors = get_anchors(anchors_path)
    # 一共有多少类
    num_classes = len(class_names)
    num_anchors = len(anchors)
    # 训练后的模型保存的位置
    log_dir = 'logs/'
    # 输入图像大小,显存比较大可以使用608x608
    input_shape = (416,416)
    # 初始epoch值
    Init_epoch = 0
    # 冻结训练的epoch值
    Freeze_epoch = 50
    # Batch_size的大小,表示每次喂入多少数据,如果OOM或者显存不足请调小
    batch_size = 8
    # 最大学习率
    learning_rate_base = 1e-3
    # 总的epoch值
    Epoch = 120;
    # 启用Mosaic数据增强
    mosaic = True
    Cosine_scheduler = False
    label_smoothing = 0
    
    # 创建验证图像保存目录
    validation_images_dir = os.path.join(log_dir, 'validation_images')
    os.makedirs(validation_images_dir, exist_ok=True)
    
    gpus = tf.config.experimental.list_physical_devices(device_type='GPU')
    for gpu in gpus:
        tf.config.experimental.set_memory_growth(gpu, True)

    # 输入的图像为
    image_input = Input(shape=(None, None, 3))
    h, w = input_shape
    # 创建yolo模型
    print('Create YOLOv4-tiny model with {} anchors and {} classes.'.format(num_anchors, num_classes))
    model_body = yolo_body(image_input, num_anchors//2, num_classes)
    
    # 获取原始类别数（6个类别）
    original_num_classes = len(class_names)
    print(f"训练目标：{original_num_classes}个类别: {class_names}")

    try:
        print('Load weights {}.'.format(weights_path))
        if os.path.exists(weights_path):
            try:
                # 先尝试直接加载
                model_body.load_weights(weights_path)
                print("成功加载权重")
            except Exception as e:
                # 检查错误信息中是否包含形状不匹配
                if "shape" in str(e):
                    print("检测到形状不匹配，可能是类别数不同")
                    # 对比打印通道数
                    loaded_shape = str(e).split("value.shape=")[1].split(")")[0]
                    expected_shape = str(e).split("variable.shape=")[1].split(")")[0]
                    print(f"期望形状：{expected_shape}，实际形状：{loaded_shape}")
                    
                    # 提取权重文件的类别数
                    weight_channels = int(loaded_shape.split(',')[-1].strip())
                    weight_num_classes = (weight_channels // 3) - 5
                    print(f"检测到权重文件对应的类别数为：{weight_num_classes}")
                    
                    # 重建模型结构以匹配权重
                    print("重新构建模型以适应权重文件...")
                    temp_model = yolo_body(image_input, num_anchors//2, weight_num_classes)
                    temp_model.load_weights(weights_path)
                    
                    # 复制共同层的权重
                    for i, layer in enumerate(model_body.layers):
                        if i < len(temp_model.layers) - 2:  # 跳过最后的输出层
                            try:
                                layer.set_weights(temp_model.layers[i].get_weights())
                                print(f"复制层 {i}: {layer.name}")
                            except:
                                print(f"跳过不兼容层 {i}: {layer.name}")
                    
                    print(f"已成功加载权重并适配到{original_num_classes}类模型")
                else:
                    raise e
        else:
            # 尝试加载COCO预训练权重
            coco_weights_path = 'model_data/yolov4_tiny_weights_coco.h5'
            print(f'未找到权重文件 {weights_path}，尝试加载COCO预训练权重 {coco_weights_path}')
            model_body.load_weights(coco_weights_path)
    except Exception as e:
        print(f'加载权重失败: {str(e)}')
        print('继续使用随机初始化的权重进行训练')

    y_true = [Input(shape=(h//{0:32, 1:16}[l], w//{0:32, 1:16}[l], num_anchors//2, num_classes+5)) for l in range(2)]

    loss_input = [*model_body.output, *y_true]
    model_loss = Lambda(yolo_loss, output_shape=(1,), name='yolo_loss',
        arguments={'anchors': anchors, 'num_classes': num_classes, 'ignore_thresh': 0.5, 'label_smoothing': label_smoothing})(loss_input)

    model = Model([model_body.input, *y_true], model_loss)

    freeze_layers = 60
    for i in range(freeze_layers): 
        model_body.layers[i].trainable = False
    print('Freeze the first {} layers of total {} layers.'.format(freeze_layers, len(model_body.layers)))

    # 训练参数设置
    logging = TensorBoard(log_dir=log_dir)
    checkpoint = ModelCheckpoint(
        log_dir + 'ep{epoch:03d}-loss{loss:.3f}-val_loss{val_loss:.3f}.weights.h5',
        monitor='val_loss', 
        save_weights_only=True, 
        save_best_only=False,  # 保存所有权重以便分析
        period=1
    )
    early_stopping = EarlyStopping(monitor='val_loss', min_delta=0, patience=6, verbose=1)

    val_split = 0.1
    with open(annotation_path) as f:
        lines = f.readlines()
    np.random.seed(10101)
    np.random.shuffle(lines)
    np.random.seed(None)
    num_val = int(len(lines)*val_split)
    num_train = len(lines) - num_val
    
    #------------------------------------------------------#
    #   主干特征提取网络特征通用，冻结训练可以加快训练速度
    #   也可以在训练初期防止权值被破坏。
    #------------------------------------------------------#
    if True:
        if Cosine_scheduler:
            # 预热期
            warmup_epoch = int((Freeze_epoch-Init_epoch)*0.2)
            # 总共的步长
            total_steps = int((Freeze_epoch-Init_epoch) * num_train / batch_size)
            # 预热步长
            warmup_steps = int(warmup_epoch * num_train / batch_size)
            # 学习率
            reduce_lr = WarmUpCosineDecayScheduler(learning_rate_base=learning_rate_base,
                                                        total_steps=total_steps,
                                                        warmup_learning_rate=1e-4,
                                                        warmup_steps=warmup_steps,
                                                        hold_base_rate_steps=num_train,
                                                        min_learn_rate=1e-6
                                                        )
            model.compile(optimizer=Adam(), loss={'yolo_loss': lambda y_true, y_pred: y_pred})
        else:
            reduce_lr = ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=2, verbose=1)
            model.compile(optimizer=Adam(learning_rate_base), loss={'yolo_loss': lambda y_true, y_pred: y_pred})

        print('Train on {} samples, val on {} samples, with batch size {}.'.format(num_train, num_val, batch_size))
        
        # 将 model.fit 的调用改为使用 tf.data.Dataset
        train_dataset = tf.data.Dataset.from_generator(
            lambda: data_generator(lines[:num_train], batch_size, input_shape, anchors, num_classes, mosaic=mosaic),
            output_signature=(
                (
                    tf.TensorSpec(shape=(batch_size, h, w, 3), dtype=tf.float32),
                    tf.TensorSpec(shape=(batch_size, h//32, w//32, num_anchors//2, num_classes+5), dtype=tf.float32),
                    tf.TensorSpec(shape=(batch_size, h//16, w//16, num_anchors//2, num_classes+5), dtype=tf.float32)
                ),
                tf.TensorSpec(shape=(batch_size,), dtype=tf.float32)
            )
        )
        
        val_dataset = tf.data.Dataset.from_generator(
            lambda: data_generator(lines[num_train:], batch_size, input_shape, anchors, num_classes, mosaic=False),
            output_signature=(
                (
                    tf.TensorSpec(shape=(batch_size, h, w, 3), dtype=tf.float32),
                    tf.TensorSpec(shape=(batch_size, h//32, w//32, num_anchors//2, num_classes+5), dtype=tf.float32),
                    tf.TensorSpec(shape=(batch_size, h//16, w//16, num_anchors//2, num_classes+5), dtype=tf.float32)
                ),
                tf.TensorSpec(shape=(batch_size,), dtype=tf.float32)
            )
        )
        
        train_dataset = train_dataset.prefetch(buffer_size=tf.data.experimental.AUTOTUNE)
        val_dataset = val_dataset.prefetch(buffer_size=tf.data.experimental.AUTOTUNE)

        #--------------------------------------------------------------#
        #   修复 Keras 在计算 dataset.cardinality() 时返回 Tensor
        #   导致 int() 转换报错的问题，强制返回 python int。
        #--------------------------------------------------------------#
        train_dataset.cardinality = lambda: -2  # tf.data.UNKNOWN_CARDINALITY
        val_dataset.cardinality   = lambda: -2

        callbacks = [logging, checkpoint, reduce_lr, early_stopping]
        
        # 添加验证可视化回调
        visualize_callback = VisualizeValidationCallback(
            val_dataset, 
            anchors, 
            class_names, 
            input_shape,
            log_dir=validation_images_dir
        )
        callbacks.append(visualize_callback)

        model.fit(train_dataset,
                steps_per_epoch=max(1, num_train//batch_size),
                validation_data=val_dataset,
                validation_steps=max(1, num_val//batch_size),
                epochs=Freeze_epoch,
                initial_epoch=Init_epoch,
                callbacks=callbacks)
        model.save_weights(log_dir + 'trained_weights_stage_1.weights.h5')

    for i in range(freeze_layers): model_body.layers[i].trainable = True
    # 解冻后训练
    if True:
        if Cosine_scheduler:
            # 预热期
            warmup_epoch = int((Epoch-Freeze_epoch)*0.2)
            # 总共的步长
            total_steps = int((Epoch-Freeze_epoch) * num_train / batch_size)
            # 预热步长
            warmup_steps = int(warmup_epoch * num_train / batch_size)
            # 学习率
            reduce_lr = WarmUpCosineDecayScheduler(learning_rate_base=learning_rate_base,
                                                        total_steps=total_steps,
                                                        warmup_learning_rate=1e-5,
                                                        warmup_steps=warmup_steps,
                                                        hold_base_rate_steps=num_train//2,
                                                        min_learn_rate=1e-6
                                                        )
            model.compile(optimizer=Adam(), loss={'yolo_loss': lambda y_true, y_pred: y_pred})
        else:
            reduce_lr = ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=2, verbose=1)
            model.compile(optimizer=Adam(learning_rate_base), loss={'yolo_loss': lambda y_true, y_pred: y_pred})

        print('Train on {} samples, val on {} samples, with batch size {}.'.format(num_train, num_val, batch_size))
        
        # 同样需要为解冻训练阶段修改
        train_dataset = tf.data.Dataset.from_generator(
            lambda: data_generator(lines[:num_train], batch_size, input_shape, anchors, num_classes, mosaic=mosaic),
            output_signature=(
                (
                    tf.TensorSpec(shape=(batch_size, h, w, 3), dtype=tf.float32),
                    tf.TensorSpec(shape=(batch_size, h//32, w//32, num_anchors//2, num_classes+5), dtype=tf.float32),
                    tf.TensorSpec(shape=(batch_size, h//16, w//16, num_anchors//2, num_classes+5), dtype=tf.float32)
                ),
                tf.TensorSpec(shape=(batch_size,), dtype=tf.float32)
            )
        )
        
        val_dataset = tf.data.Dataset.from_generator(
            lambda: data_generator(lines[num_train:], batch_size, input_shape, anchors, num_classes, mosaic=False),
            output_signature=(
                (
                    tf.TensorSpec(shape=(batch_size, h, w, 3), dtype=tf.float32),
                    tf.TensorSpec(shape=(batch_size, h//32, w//32, num_anchors//2, num_classes+5), dtype=tf.float32),
                    tf.TensorSpec(shape=(batch_size, h//16, w//16, num_anchors//2, num_classes+5), dtype=tf.float32)
                ),
                tf.TensorSpec(shape=(batch_size,), dtype=tf.float32)
            )
        )
        
        train_dataset = train_dataset.prefetch(buffer_size=tf.data.experimental.AUTOTUNE)
        val_dataset = val_dataset.prefetch(buffer_size=tf.data.experimental.AUTOTUNE)

        # 同样修复 cardinality()
        train_dataset.cardinality = lambda: -2
        val_dataset.cardinality   = lambda: -2

        callbacks = [logging, checkpoint, reduce_lr, early_stopping]
        
        # 添加验证可视化回调（解冻训练阶段）
        visualize_callback = VisualizeValidationCallback(
            val_dataset, 
            anchors, 
            class_names, 
            input_shape,
            log_dir=validation_images_dir
        )
        callbacks.append(visualize_callback)

        model.fit(train_dataset,
                steps_per_epoch=max(1, num_train//batch_size),
                validation_data=val_dataset,
                validation_steps=max(1, num_val//batch_size),
                epochs=Epoch,
                initial_epoch=Freeze_epoch,
                callbacks=callbacks)
        model.save_weights(log_dir + 'last1.weights.h5')
