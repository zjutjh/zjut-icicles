import tensorflow as tf
from tensorflow.keras import backend as K
from yolo_nets.ious import box_ciou

#---------------------------------------------------#
#   平滑标签
#---------------------------------------------------#
def _smooth_labels(y_true, label_smoothing):
    num_classes = tf.cast(K.shape(y_true)[-1], dtype=K.floatx())
    label_smoothing = K.constant(label_smoothing, dtype=K.floatx())
    return y_true * (1.0 - label_smoothing) + label_smoothing / num_classes
#---------------------------------------------------#
#   将预测值的每个特征层调成真实值
#---------------------------------------------------#
def yolo_head(feats, anchors, num_classes, input_shape, calc_loss=False):
    num_anchors = len(anchors)
    # [1, 1, 1, num_anchors, 2]
    anchors_tensor = K.reshape(K.constant(anchors), [1, 1, 1, num_anchors, 2])

    # 获得x，y的网格
    # (13, 13, 1, 2)
    grid_shape = K.shape(feats)[1:3] # height, width
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
#   用于计算每个预测框与真实框的iou
#---------------------------------------------------#
def box_iou(b1, b2):
    # 13,13,3,1,4
    # 计算左上角的坐标和右下角的坐标
    b1 = K.expand_dims(b1, -2)
    b1_xy = b1[..., :2]
    b1_wh = b1[..., 2:4]
    b1_wh_half = b1_wh/2.
    b1_mins = b1_xy - b1_wh_half
    b1_maxes = b1_xy + b1_wh_half

    # 1,n,4
    # 计算左上角和右下角的坐标
    b2 = K.expand_dims(b2, 0)
    b2_xy = b2[..., :2]
    b2_wh = b2[..., 2:4]
    b2_wh_half = b2_wh/2.
    b2_mins = b2_xy - b2_wh_half
    b2_maxes = b2_xy + b2_wh_half

    # 计算重合面积
    intersect_mins = K.maximum(b1_mins, b2_mins)
    intersect_maxes = K.minimum(b1_maxes, b2_maxes)
    intersect_wh = K.maximum(intersect_maxes - intersect_mins, 0.)
    intersect_area = intersect_wh[..., 0] * intersect_wh[..., 1]
    b1_area = b1_wh[..., 0] * b1_wh[..., 1]
    b2_area = b2_wh[..., 0] * b2_wh[..., 1]
    iou = intersect_area / (b1_area + b2_area - intersect_area)

    return iou


#---------------------------------------------------#
#   loss值计算
#---------------------------------------------------#
def yolo_loss(args, anchors, num_classes, ignore_thresh=.5, label_smoothing=0.1, print_loss=False):

    # 一共有2层
    num_layers = len(anchors)//3

    # 将预测结果和实际ground truth分开，args是[*model_body.output, *y_true]
    # y_true是一个列表，包含两个特征层，shape分别为(m,13,13,3,85),(m,26,26,3,85)。
    # yolo_outputs是一个列表，包含两个特征层，shape分别为(m,13,13,255),(m,26,26,255)。
    y_true = args[num_layers:]
    yolo_outputs = args[:num_layers]

    # 先验框
    anchor_mask = [[6,7,8], [3,4,5], [0,1,2]] if num_layers==3 else [[3,4,5], [1,2,3]]

    # 得到input_shpae为608,608 
    input_shape = K.cast(K.shape(yolo_outputs[0])[1:3] * 32, K.dtype(y_true[0]))

    loss = 0

    # 取出每一张图片
    # m的值就是batch_size
    m = K.shape(yolo_outputs[0])[0]
    mf = K.cast(m, K.dtype(yolo_outputs[0]))

    # y_true是一个列表，包含两个特征层，shape分别为(m,13,13,3,85),(m,26,26,3,85)。
    # yolo_outputs是一个列表，包含两个特征层，shape分别为(m,13,13,255),(m,26,26,255)。
    for l in range(num_layers):
        # 以第一个特征层(m,13,13,3,85)为例子
        # 取出该特征层中存在目标的点的位置。(m,13,13,3,1)
        object_mask = y_true[l][..., 4:5]
        # 取出其对应的种类(m,13,13,3,80)
        true_class_probs = y_true[l][..., 5:]
        if label_smoothing:
            true_class_probs = _smooth_labels(true_class_probs, label_smoothing)

        # 将yolo_outputs的特征层输出进行处理
        # grid为网格结构(13,13,1,2)，raw_pred为尚未处理的预测结果(m,13,13,3,85)
        # 还有解码后的xy，wh，(m,13,13,3,2)
        grid, raw_pred, pred_xy, pred_wh = yolo_head(yolo_outputs[l],
             anchors[anchor_mask[l]], num_classes, input_shape, calc_loss=True)
        
        # 这个是解码后的预测的box的位置
        # (m,13,13,3,4)
        pred_box = K.concatenate([pred_xy, pred_wh])

        # 找到负样本群组，第一步是创建一个数组，[]
        ignore_mask = tf.TensorArray(K.dtype(y_true[0]), size=1, dynamic_size=True)
        object_mask_bool = K.cast(object_mask, 'bool')
        
        # 对每一张图片计算ignore_mask
        def loop_body(b, ignore_mask):
            # 取出第b副图内，真实存在的所有的box的参数
            # n,4
            true_box = tf.boolean_mask(y_true[l][b,...,0:4], object_mask_bool[b,...,0])
            # 计算预测结果与真实情况的iou
            # pred_box为13,13,3,4
            # 计算的结果是每个pred_box和其它所有真实框的iou
            # 13,13,3,n
            iou = box_iou(pred_box[b], true_box)

            # 13,13,3
            best_iou = K.max(iou, axis=-1)

            # 如果某些预测框和真实框的重合程度大于0.5，则忽略。
            ignore_mask = ignore_mask.write(b, K.cast(best_iou<ignore_thresh, K.dtype(true_box)))
            return b+1, ignore_mask

        # 遍历所有的图片
        _, ignore_mask = tf.while_loop(lambda b,*args: b<m, loop_body, [0, ignore_mask])

        # 将每幅图的内容压缩，进行处理
        ignore_mask = ignore_mask.stack()
        #(m,13,13,3,1)
        ignore_mask = K.expand_dims(ignore_mask, -1)

        box_loss_scale = 2 - y_true[l][...,2:3]*y_true[l][...,3:4]

        # Calculate ciou loss as location loss
        raw_true_box = y_true[l][...,0:4]
        ciou = box_ciou(pred_box, raw_true_box)
        ciou_loss = object_mask * box_loss_scale * (1 - ciou)
        ciou_loss = K.sum(ciou_loss) / mf
        location_loss = ciou_loss
        
        # 如果该位置本来有框，那么计算1与置信度的交叉熵
        # 如果该位置本来没有框，而且满足best_iou<ignore_thresh，则被认定为负样本
        # best_iou<ignore_thresh用于限制负样本数量
        confidence_loss = object_mask * K.binary_crossentropy(object_mask, raw_pred[...,4:5], from_logits=True)+ \
            (1-object_mask) * K.binary_crossentropy(object_mask, raw_pred[...,4:5], from_logits=True) * ignore_mask
        
        class_loss = object_mask * K.binary_crossentropy(true_class_probs, raw_pred[...,5:], from_logits=True)

        confidence_loss = K.sum(confidence_loss) / mf
        class_loss = K.sum(class_loss) / mf
        loss += location_loss + confidence_loss + class_loss
        # if print_loss:
        # loss = tf.Print(loss, [loss, confidence_loss, class_loss, location_loss], message='loss: ')
    loss = K.expand_dims(loss, axis=-1)
    return loss
