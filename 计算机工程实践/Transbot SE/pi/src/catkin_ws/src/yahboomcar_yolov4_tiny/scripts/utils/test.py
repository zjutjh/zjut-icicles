from yolo_nets.yolo4_tiny import yolo_body
from tensorflow.keras.layers import Input

# 输入的图像为
image_input = Input(shape=(416, 416, 3))
model = yolo_body(image_input,3,20)
model.summary()

for i,layer in enumerate(model.layers):
    print(i,layer.name)