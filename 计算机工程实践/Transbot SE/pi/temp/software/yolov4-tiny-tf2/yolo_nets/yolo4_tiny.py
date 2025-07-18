from functools import wraps

import numpy as np
import tensorflow as tf
from tensorflow.keras import backend as K
from tensorflow.keras.layers import Conv2D, Add, ZeroPadding2D, UpSampling2D, Concatenate, MaxPooling2D, LeakyReLU, \
    BatchNormalization, Input, Lambda
from tensorflow.keras.models import Model
from tensorflow.keras.regularizers import l2
from yolo_nets.CSPdarknet53_tiny import darknet_body
from utils.utils import compose



#--------------------------------------------------#
#   单次卷积
#--------------------------------------------------#
@wraps(Conv2D)
def DarknetConv2D(*args, **kwargs):
    darknet_conv_kwargs = {'kernel_regularizer': l2(5e-4)}
    darknet_conv_kwargs['padding'] = 'valid' if kwargs.get('strides')==(2,2) else 'same'
    darknet_conv_kwargs.update(kwargs)
    return Conv2D(*args, **darknet_conv_kwargs)

#---------------------------------------------------#
#   卷积块
#   DarknetConv2D + BatchNormalization + LeakyReLU
#---------------------------------------------------#
def DarknetConv2D_BN_Leaky(*args, **kwargs):
    no_bias_kwargs = {'use_bias': False}
    no_bias_kwargs.update(kwargs)
    return compose( 
        DarknetConv2D(*args, **no_bias_kwargs),
        BatchNormalization(),
        LeakyReLU(alpha=0.1))

#---------------------------------------------------#
#   特征层->最后的输出
#---------------------------------------------------#
def yolo_body(inputs, num_anchors, num_classes):
    # 生成darknet53的主干模型
    # 首先我们会获取到两个有效特征层
    # feat1 26x26x256
    # feat2 13x13x512
    feat1,feat2 = darknet_body(inputs)

    # 13x13x512 -> 13x13x256
    P5 = DarknetConv2D_BN_Leaky(256, (1,1))(feat2)

    P5_output = DarknetConv2D_BN_Leaky(512, (3,3))(P5)
    P5_output = DarknetConv2D(num_anchors*(num_classes+5), (1,1))(P5_output)
    
    # Conv+UpSampling2D 13x13x256 -> 26x26x128
    P5_upsample = compose(DarknetConv2D_BN_Leaky(128, (1,1)), UpSampling2D(2))(P5)
    
    # 26x26x(128+256) 26x26x384
    P4 = Concatenate()([feat1, P5_upsample])
    
    P4_output = DarknetConv2D_BN_Leaky(256, (3,3))(P4)
    P4_output = DarknetConv2D(num_anchors*(num_classes+5), (1,1))(P4_output)
    
    return Model(inputs, [P5_output, P4_output])

#---------------------------------------------------#
#   将预测值的每个特征层调成真实值
#---------------------------------------------------#
def yolo_head(feats, anchors, num_classes, input_shape, calc_loss=False):
    num_anchors = len(anchors)
    # [1, 1, 1, num_anchors, 2]
    anchors_tensor = K.reshape(K.constant(anchors), [1, 1, 1, num_anchors, 2])

    grid_shape = Lambda(lambda x: K.shape(x)[1:3])(feats)  # height, width
    grid_y = K.tile(K.reshape(K.arange(0, stop=grid_shape[0]), [-1, 1, 1, 1]),
        [1, grid_shape[1], 1, 1])
    grid_x = K.tile(K.reshape(K.arange(0, stop=grid_shape[1]), [1, -1, 1, 1]),
        [grid_shape[0], 1, 1, 1])
    grid = K.concatenate([grid_x, grid_y])
    grid = K.cast(grid, K.dtype(feats))

    # (batch_size,13,13,3,85)
    feats = K.reshape(feats, [-1, grid_shape[0], grid_shape[1], num_anchors, num_classes + 5])

    # 将预测值调成真实值
    # box_xy对应框的中心点
    # box_wh对应框的宽和高
    box_xy = (K.sigmoid(feats[..., :2]) + grid) / K.cast(grid_shape[...,::-1], K.dtype(feats))
    box_wh = K.exp(feats[..., 2:4]) * anchors_tensor / K.cast(input_shape[...,::-1], K.dtype(feats))
    box_confidence = K.sigmoid(feats[..., 4:5])
    box_class_probs = K.sigmoid(feats[..., 5:])

    # 在计算loss的时候返回如下参数
    if calc_loss == True:
        return grid, feats, box_xy, box_wh
    return box_xy, box_wh, box_confidence, box_class_probs

#---------------------------------------------------#
#   对box进行调整，使其符合真实图片的样子
#---------------------------------------------------#
def yolo_correct_boxes(box_xy, box_wh, input_shape, image_shape):
    box_yx = box_xy[..., ::-1]
    box_hw = box_wh[..., ::-1]
    
    input_shape = K.cast(input_shape, K.dtype(box_yx))
    image_shape = K.cast(image_shape, K.dtype(box_yx))

    new_shape = K.round(image_shape * K.min(input_shape/image_shape))
    offset = (input_shape-new_shape)/2./input_shape
    scale = input_shape/new_shape

    box_yx = (box_yx - offset) * scale
    box_hw *= scale

    box_mins = box_yx - (box_hw / 2.)
    box_maxes = box_yx + (box_hw / 2.)
    boxes =  K.concatenate([
        box_mins[..., 0:1],  # y_min
        box_mins[..., 1:2],  # x_min
        box_maxes[..., 0:1],  # y_max
        box_maxes[..., 1:2]  # x_max
    ])

    boxes *= K.concatenate([image_shape, image_shape])
    return boxes

#---------------------------------------------------#
#   获取每个box和它的得分
#---------------------------------------------------#
def yolo_boxes_and_scores(feats, anchors, num_classes, input_shape, image_shape):
    """Process Conv layer output to get boxes and scores."""
    
    def yolo_head_lambda(args):
        """
        The core logic of yolo_head, now wrapped in a function for a Lambda layer.
        It takes feats and input_shape as symbolic tensors and processes them.
        """
        feats_tensor, input_shape_tensor = args
        num_anchors_ = len(anchors)
        
        anchors_tensor = K.reshape(K.constant(anchors), [1, 1, 1, num_anchors_, 2])

        grid_shape = K.shape(feats_tensor)[1:3]  # height, width
        grid_y = K.tile(K.reshape(K.arange(0, stop=grid_shape[0]), [-1, 1, 1, 1]),
                        [1, grid_shape[1], 1, 1])
        grid_x = K.tile(K.reshape(K.arange(0, stop=grid_shape[1]), [1, -1, 1, 1]),
                        [grid_shape[0], 1, 1, 1])
        grid = K.concatenate([grid_x, grid_y])
        grid = K.cast(grid, K.dtype(feats_tensor))

        feats_reshaped = K.reshape(
            feats_tensor, [-1, grid_shape[0], grid_shape[1], num_anchors_, num_classes + 5])

        box_xy = (K.sigmoid(feats_reshaped[..., :2]) + grid) / K.cast(grid_shape[::-1], K.dtype(feats_tensor))
        box_wh = K.exp(feats_reshaped[..., 2:4]) * anchors_tensor / K.cast(input_shape_tensor, K.dtype(feats_tensor))
        box_confidence = K.sigmoid(feats_reshaped[..., 4:5])
        box_class_probs = K.sigmoid(feats_reshaped[..., 5:])
        
        return box_xy, box_wh, box_confidence, box_class_probs

    box_xy, box_wh, box_confidence, box_class_probs = Lambda(yolo_head_lambda, name='yolo_head')([feats, input_shape])

    boxes = yolo_correct_boxes(box_xy, box_wh, input_shape, image_shape)
    boxes = K.reshape(boxes, [-1, 4])
    box_scores = box_confidence * box_class_probs
    box_scores = K.reshape(box_scores, [-1, num_classes])
    return boxes, box_scores

# ---------------------------------------------------#
#   图片预测
# ---------------------------------------------------#
def yolo_eval(args, anchors, num_classes, score_threshold=.01, iou_threshold=.5):
    """
    Evaluation YOLOv4-tiny model on given input and return filtered boxes.
    This function is designed to be used in a Lambda layer.
    """
    print(f"yolo_eval: 锚框数量 {len(anchors)}, 类别数 {num_classes}, 阈值 {score_threshold}")
    yolo_outputs = args[:-1]
    image_shape = args[-1]

    num_layers = len(yolo_outputs)
    # YOLOv4-tiny有两个输出层，每层使用3个锚框
    # 注意：anchor_mask应调整为[[3,4,5], [1,2,3]]而非[[3,4,5], [0,1,2]]
    anchor_mask = [[3,4,5], [1,2,3]] 
    
    input_shape = K.shape(yolo_outputs[0])[1:3] * 32
    
    boxes = []
    box_scores = []
    for l in range(num_layers):
        _boxes, _box_scores = yolo_boxes_and_scores(yolo_outputs[l], anchors[anchor_mask[l]], num_classes, input_shape, image_shape)
        boxes.append(_boxes)
        box_scores.append(_box_scores)
    boxes = K.concatenate(boxes, axis=0)
    box_scores = K.concatenate(box_scores, axis=0)

    mask = box_scores >= score_threshold
    max_boxes_tensor = K.constant(20, dtype='int32')
    boxes_ = []
    scores_ = []
    classes_ = []
    for c in range(num_classes):
        class_boxes = tf.boolean_mask(boxes, mask[:, c])
        class_box_scores = tf.boolean_mask(box_scores[:, c], mask[:, c])
        nms_index = tf.image.non_max_suppression(
            class_boxes, class_box_scores, max_boxes_tensor, iou_threshold=iou_threshold)
        class_boxes = K.gather(class_boxes, nms_index)
        class_box_scores = K.gather(class_box_scores, nms_index)
        classes = K.ones_like(class_box_scores, 'int32') * c
        boxes_.append(class_boxes)
        scores_.append(class_box_scores)
        classes_.append(classes)
    boxes_ = K.concatenate(boxes_, axis=0)
    scores_ = K.concatenate(scores_, axis=0)
    classes_ = K.concatenate(classes_, axis=0)

    return boxes_, scores_, classes_
