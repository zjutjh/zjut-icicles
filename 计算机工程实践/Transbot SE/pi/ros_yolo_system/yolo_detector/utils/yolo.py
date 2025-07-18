import os
import colorsys
import numpy as np
import tensorflow as tf
from timeit import default_timer as timer
from tensorflow.keras import backend as K
from tensorflow.keras.layers import Input, Lambda
from tensorflow.keras.models import Model
from PIL import ImageFont, ImageDraw
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.callbacks import TensorBoard, ModelCheckpoint, ReduceLROnPlateau, EarlyStopping
from yolo_nets.yolo4_tiny import yolo_body, yolo_eval
from utils.utils import letterbox_image


class YOLO(object):
    _defaults = {
        "model_path": 'model_data/last1.weights.h5',
        "classes_path": 'model_data/CampusBot.txt',
        "anchors_path": 'model_data/CampusBot_anchors.txt',
        "anchors_mask": [[3, 4, 5], [1, 2, 3]],
        "input_shape": [416, 416],
        "confidence": 0.3,
        "nms_iou": 0.3,
        "max_boxes": 100,
        "letterbox_image": True,
        "score": 0.5,
        "iou": 0.3,
        "eager": True,
        # 显存比较小可以使用tiny模型
        "model_image_size": (416, 416)
    }

    @classmethod
    def get_defaults(cls, n):
        if n in cls._defaults:
            return cls._defaults[n]
        else:
            return "Unrecognized attribute name '" + n + "'"

    # ---------------------------------------------------#
    #   初始化yolo
    # ---------------------------------------------------#
    def __init__(self, **kwargs):
        self.font_path = 'font/Block_Simplified.TTF'
        self.__dict__.update(self._defaults)
        
        # 使用kwargs中的参数覆盖默认值
        for key, value in kwargs.items():
            if key in self._defaults:
                setattr(self, key, value)
                print(f"设置参数 {key} = {value}")
        
        self.class_names = self._get_class()
        self.anchors = self._get_anchors()
        
        # Generate colors for drawing bounding boxes.
        hsv_tuples = [(x / len(self.class_names), 1., 1.)
                      for x in range(len(self.class_names))]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)),
                self.colors))

        # Build pure Keras model for inference
        self.yolo_model = self._build_inference_model()

    # ---------------------------------------------------#
    #   获得所有的分类
    # ---------------------------------------------------#
    def _get_class(self):
        classes_path = os.path.expanduser(self.classes_path)
        with open(classes_path) as f:
            class_names = f.readlines()
        class_names = [c.strip() for c in class_names]
        return class_names

    # ---------------------------------------------------#
    #   获得所有的先验框
    # ---------------------------------------------------#
    def _get_anchors(self):
        anchors_path = os.path.expanduser(self.anchors_path)
        with open(anchors_path) as f:
            anchors = f.readline()
        anchors = [float(x) for x in anchors.split(',')]
        return np.array(anchors).reshape(-1, 2)

    def _build_inference_model(self):
        model_path = os.path.expanduser(self.model_path)
        assert model_path.endswith('.h5'), 'Keras model or weights must be a .h5 file.'
        
        num_anchors = len(self.anchors)
        num_classes = len(self.class_names)
        
        print(f"加载模型，使用类别数: {num_classes}, 锚框数: {num_anchors}")
        print(f"使用参数：score_threshold={self.score}, iou_threshold={self.iou}")
        
        try:
            # 首先尝试直接加载完整模型
            print('尝试直接加载完整模型...')
            yolo_model = tf.keras.models.load_model(model_path, compile=False)
            print('{} 模型加载成功.'.format(model_path))
        except Exception as e:
            print(f'加载完整模型失败: {str(e)}')
            print('正在创建模型结构并加载权重...')
            
            try:
                # 创建模型结构
                base_model = yolo_body(Input(shape=(None,None,3)), num_anchors//2, num_classes)
                
                # 如果权重文件存在，尝试加载
                if os.path.exists(model_path):
                    print(f'尝试加载权重: {model_path}')
                    # 修改这里：移除by_name参数，尝试直接加载权重
                    base_model.load_weights(model_path)
                    print('权重加载完成')
                else:
                    print(f'警告: 权重文件 {model_path} 不存在，使用随机初始化的权重')
                
                yolo_model = base_model
            except Exception as e:
                print(f'模型创建或权重加载失败: {str(e)}')
                raise e
        
        # 构建完整的推理模型，包括后处理
        input_image_shape = Input(shape=(2, ), name='image_shape', dtype=tf.float32)
        
        # 确保使用正确的阈值
        print(f"传递给yolo_eval的参数: score_threshold={self.score}, iou_threshold={self.iou}")
        outputs = Lambda(yolo_eval, name='yolo_eval', arguments={
            'anchors': self.anchors, 
            'num_classes': num_classes,
            'score_threshold': float(self.score),  # 确保是float类型
            'iou_threshold': float(self.iou)
        })([*yolo_model.output, input_image_shape])
        
        inference_model = Model([yolo_model.input, input_image_shape], outputs)
        return inference_model

    # ---------------------------------------------------#
    #   检测图片
    # ---------------------------------------------------#
    def detect_image(self, image):
        start = timer()

        # 调整图片使其符合输入要求
        new_image_size = self.model_image_size
        boxed_image = letterbox_image(image, new_image_size)
        image_data = np.array(boxed_image, dtype='float32')
        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

        input_image_shape = np.expand_dims(np.array([image.size[1], image.size[0]], dtype='float32'), 0)
        
        out_boxes, out_scores, out_classes = self.yolo_model.predict([image_data, input_image_shape])

        print('Found {} boxes for {}'.format(len(out_boxes), 'img'))
        
        # 创建结果的副本用于返回
        boxes_return = out_boxes.copy()
        scores_return = out_scores.copy()
        classes_return = out_classes.copy()
        
        for i, c in list(enumerate(out_classes)):
            predicted_class = self.class_names[c]
            box = out_boxes[i]
            score = out_scores[i]
            print('predicted_class: {}'.format(predicted_class))
            self.draw_img(image, c, box, score, predicted_class)
        end = timer()
        print(end - start)
        return image, boxes_return, scores_return, classes_return

    def draw_img(self, image, c, box, score, predicted_class):
        top, left, bottom, right = box
        top = top - 5
        left = left - 5
        bottom = bottom + 5
        right = right + 5
        top = max(0, np.floor(top + 0.5).astype('int32'))
        left = max(0, np.floor(left + 0.5).astype('int32'))
        bottom = min(image.size[1], np.floor(bottom + 0.5).astype('int32'))
        right = min(image.size[0], np.floor(right + 0.5).astype('int32'))
        label = '{} {:.2f}'.format(predicted_class, score)
        # 设置字体
        try:
            font = ImageFont.truetype(font=self.font_path,
                                      size=np.floor(3e-2 * image.size[1] + 0.5).astype('int32'))
        except:
            font = ImageFont.load_default()
            
        thickness = (image.size[0] + image.size[1]) // 300
        draw = ImageDraw.Draw(image)
        
        # 获取文本尺寸，处理可能的API变化
        try:
            label_size = draw.textsize(label, font)
        except:
            try:
                # 尝试新版PIL的API
                label_size = (draw.textlength(label, font), font.getsize(label)[1])
            except:
                # 如果都失败，估算一个合理的尺寸
                label_size = (len(label) * 10, 20)
                
        label_unicode = label
        
        # 绘制边界框
        if top - label_size[1] >= 0:
            text_origin = np.array([left, top - label_size[1]])
        else:
            text_origin = np.array([left, top + 1])
        for i in range(thickness):
            draw.rectangle(
                [left + i, top + i, right - i, bottom - i],
                outline=self.colors[c])
        draw.rectangle(
            [tuple(text_origin), tuple(text_origin + label_size)],
            fill=self.colors[c])
        draw.text(text_origin, label_unicode, fill=(0, 0, 0), font=font)
        del draw
