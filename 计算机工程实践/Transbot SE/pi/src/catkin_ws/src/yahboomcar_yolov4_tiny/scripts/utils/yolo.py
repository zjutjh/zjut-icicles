import os
import colorsys
import numpy as np
import tensorflow as tf
from timeit import default_timer as timer
from tensorflow.compat.v1.keras import backend as K
from tensorflow.keras.layers import Input
from tensorflow.keras.layers import Lambda
from tensorflow.keras.models import Model
from PIL import ImageFont, ImageDraw, Image
from yolo_nets.yolo4_tiny import yolo_body, yolo_eval
from utils.utils import letterbox_image


class YOLO(object):
    _defaults = {
        "score": 0.5,
        "iou": 0.3,
        "eager": False,
        # 显存比较小可以使用416x416
        # 显存比较大可以使用608x608
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
    def __init__(self, rospkg_path, model_path, classes_path, **kwargs):
        self.model_path = model_path
        self.classes_path = classes_path
        self.font_path = rospkg_path + '/font/Block_Simplified.TTF'
        self.anchors_path = rospkg_path + '/param/yolo_anchors.txt'
        self.__dict__.update(self._defaults)
        self.class_names = self._get_class()
        self.anchors = self._get_anchors()
        if not self.eager:
            tf.compat.v1.disable_eager_execution()
            self.sess = K.get_session()
        self.generate()

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

    # ---------------------------------------------------#
    #   获得所有的分类
    # ---------------------------------------------------#
    def generate(self):
        model_path = os.path.expanduser(self.model_path)
        assert model_path.endswith('.h5'), 'Keras model or weights must be a .h5 file.'
        # 计算anchor数量
        num_anchors = len(self.anchors)
        num_classes = len(self.class_names)
        # 载入模型，如果原来的模型里已经包括了模型结构则直接载入。
        # 否则先构建模型再载入
        self.yolo_model = yolo_body(Input(shape=(None, None, 3)), num_anchors // 2, num_classes)
        self.yolo_model.load_weights(self.model_path)
        print('{} model, anchors, and classes loaded.'.format(model_path))
        # 画框设置不同的颜色
        hsv_tuples = [(x / len(self.class_names), 1., 1.)
                      for x in range(len(self.class_names))]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)),
                self.colors))
        # 打乱颜色
        np.random.seed(10101)
        np.random.shuffle(self.colors)
        np.random.seed(None)
        if self.eager:
            self.input_image_shape = Input([2, ], batch_size=1)
            inputs = [*self.yolo_model.output, self.input_image_shape]
            outputs = Lambda(yolo_eval, output_shape=(1,), name='yolo_eval',
                             arguments={'anchors': self.anchors, 'num_classes': len(self.class_names),
                                        'image_shape': self.model_image_size,
                                        'score_threshold': self.score, 'eager': True})(inputs)
            self.yolo_model = Model([self.yolo_model.input, self.input_image_shape], outputs)
        else:
            self.input_image_shape = K.placeholder(shape=(2,))
            self.boxes, self.scores, self.classes = yolo_eval(self.yolo_model.output, self.anchors,
                                                              num_classes, self.input_image_shape,
                                                              score_threshold=self.score, iou_threshold=self.iou)

    # ---------------------------------------------------#
    #   检测图片
    # ---------------------------------------------------#
    def detect_image(self, image):

        # 转变成Image
        image = Image.fromarray(np.uint8(image))
        # 调整图片使其符合输入要求
        boxed_image = letterbox_image(image, self.model_image_size)
        image_data = np.array(boxed_image, dtype='float32')
        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.
        if self.eager:
            # 预测结果
            input_image_shape = np.expand_dims(np.array([image.size[1], image.size[0]], dtype='float32'), 0)
            out_boxes, out_scores, out_classes = self.yolo_model.predict([image_data, input_image_shape])
        else:
            # 预测结果
            out_boxes, out_scores, out_classes = self.sess.run(
                [self.boxes, self.scores, self.classes],
                feed_dict={
                    self.yolo_model.input: image_data,
                    self.input_image_shape: [image.size[1], image.size[0]],
                    K.learning_phase(): 0
                })
        # print('out_boxes: {}, out_scores: {}, out_classes: {}'.format(out_boxes, out_scores, out_classes))
        return image, out_boxes, out_scores, out_classes

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
        font = ImageFont.truetype(
            font=self.font_path, size=np.floor(3e-2 * image.size[1] + 0.5).astype('int32'))
        thickness = (image.size[0] + image.size[1]) // 300
        draw = ImageDraw.Draw(image)
        label_size = draw.textsize(label, font)
        label = label.encode('utf-8')
        # print(label)
        if top - label_size[1] >= 0: text_origin = np.array([left, top - label_size[1]])
        else: text_origin = np.array([left, top + 1])
        for i in range(thickness):
            draw.rectangle(
                [left + i, top + i, right - i, bottom - i],
                outline=self.colors[c])
        draw.rectangle(
            [tuple(text_origin), tuple(text_origin + label_size)],
            fill=self.colors[c])
        draw.text(text_origin, str(label, 'UTF-8'), fill=(0, 0, 0), font=font)
        del draw

