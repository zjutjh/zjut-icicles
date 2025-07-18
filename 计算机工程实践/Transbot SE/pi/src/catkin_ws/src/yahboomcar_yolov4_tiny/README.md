### 1.环境要求

```
tensorflow-gpu==2.2.0
lxml
matplotlib 
pandas 
Pillow
scikit-learn
seaborn
tqdm
imgaug

pip install tqdm seaborn scikit-learn Pillow pandas lxml matplotlib imgaug

# pip install h5py==2.10 typing-extensions==3.7.4 six==1.15.0 grpcio==1.32.0 absl-py==0.10
# pip uninstall h5py absl-py grpcio six typing-extensions

```

### 2.模型训练流程

##### (1)文件夹结构

```文件夹结构
garbage_data: 存放数据集
garbage_data/image: 目标源文件
garbage_data/JPEGImages: 数据集图片(尽可能多些)
garbage_data/texture: 背景图片(尽可能多些)
garbage_data/train.txt: 与数据集图片相对应的标签文件
garbage_data/GetData.py: 获取数据集代码
font: 存放字体包
img: 存放测试图片
logs: 存放测试日志以及最终的训练模型last1.h5
model_data: 存放预训练模型(权重文件),自定义标签文件(与目标源文件对应),yolo的模型参数anchors
nets 和 utils: yolo的一些库文件
```

```anchor简介
在YOLO-v2版本中就引入了anchor box的概念，极大增加了目标检测的性能,
anchor的本质就是SPP(spatial pyramid pooling)思想的逆向,
而SPP本身是做什么的呢,就是将不同尺寸的输入resize成为相同尺寸的输出,
所以SPP的逆向就是将相同尺寸的输出,倒推得到不同尺寸的输入.
```

##### (2)训练步骤

训练代码来源网络,原文链接https://github.com/bubbliiiing/yolov4-tiny-tf2

- 制作数据集


图片和标签文件的名字要对应上，train.txt文件中的标签格式如下 :   

```
./garbage_data/JPEGImages/0.jpg 113,163,293,298,9
# 图片路径      y, x, y + w, x + h ,label
```

```数据集制作方法
数据集制作方法:
一种方法是先拍一些照片,使用标注工具对每张照片上的目标进行标注,在garbage_data文件夹下新建train.txt文件,将目标信息写入.
另一种方法在garbage_data/texture文件夹中放入背景图片(尽可能多些),根据需求修改GetData.py代码,执行GetData.py生成数据集(尽可能多些)
```
- 添加权重文件

最新的权重文件百度搜索下载即可,在garbage_data文件下有提供好的权重文件yolov4_tiny_weights_coco.h5和yolov4_tiny_weights_voc.h5

- 制作自己的classes--->garbage.txt

注意最好不要使用中文标签，文件夹中不要有空格！

```
Zip_top_can
Old_school_bag
Newspaper
Book
Toilet_paper
......
```

- 修改train.py文件

根据自己的需求,参考注释进行修改.

```
# 标签的位置
annotation_path = 'garbage_data/train.txt'
# 获取classes和anchor的位置
classes_path = 'model_data/garbage.txt'
anchors_path = 'model_data/yolo_anchors.txt'
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
batch_size = 16
# 最大学习率
learning_rate_base = 1e-3
# 总的epoch值
Epoch = 100
```

- 开始训练

按照上述流程,操作完毕后,直接运行train.py文件训练即可.

##### (3)自定义模型检测

- 修改yolo.py文件

```
class YOLO(object):
    _defaults = {
        "model_path": 'model_data/garbage.h5',
        "anchors_path": 'model_data/yolo_anchors.txt',
        "classes_path": 'model_data/garbage.txt',
        "score" : 0.5,
        "iou" : 0.3,
        "eager" : False,
        # 默认使用416x416(图片大小)
        "model_image_size" : (416, 416)
    }
... ...
self.font_path = 'font/Block_Simplified.TTF'
```

model_path: 用于检测,训练好的模型路径(ROS环境下需用全局路径)

anchors_path: yolo的模型参数anchors路径(ROS环境下需用全局路径)

classes_path: 自定义标签文件路径(ROS环境下需用全局路径)

self.font_path: 字体包路径(ROS环境下需用全局路径)

- 执行py文件检测

predict_img.py: 图片检测

predict_video.py: 视频检测

### 