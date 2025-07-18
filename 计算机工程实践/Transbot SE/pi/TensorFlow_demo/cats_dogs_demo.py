#导入相关模块
# Import related modules
import tensorflow as tf
import os
import numpy as np
import matplotlib.pyplot as plt
from tensorflow.keras import backend as K


K.clear_session()
#训练集和测试集目录
#file directory 
base_dir = './dataset1/'
train_dir = os.path.join(base_dir, 'training_set/')
validation_dir = os.path.join(base_dir, 'test_set/')
train_cats_dir = os.path.join(train_dir, 'cats')  
train_dogs_dir = os.path.join(train_dir, 'dogs')  
validation_cats_dir = os.path.join(validation_dir, 'cats')  
validation_dogs_dir = os.path.join(validation_dir, 'dogs') 
 
num_cats_tr = len(os.listdir(train_cats_dir))  # total training cat images: 4000
num_dogs_tr = len(os.listdir(train_dogs_dir))  # total training dog images: 4000
print("num_cats_tr:",num_cats_tr)
print("num_dogs_tr:",num_dogs_tr)
 
num_cats_val = len(os.listdir(validation_cats_dir))  # total validation cat images: 1000
num_dogs_val = len(os.listdir(validation_dogs_dir))  # total validation dog images: 1000
print("num_cats_val:",num_cats_val)
print("num_dogs_val:",num_dogs_val)
 
total_train = num_cats_tr + num_dogs_tr  # Total training images: 8000
total_val = num_cats_val + num_dogs_val  # Total validation images: 2000
print("total_train:",total_train)
print("total_val:",total_val)

#初始化配置数据
#initialization data 
batch_size = 64
epochs = 10
IMG_HEIGHT = 150
IMG_WIDTH = 150

#生成我们的培训和验证数据集
# Generate our training and validation datasets 
train_image_generator = tf.keras.preprocessing.image.ImageDataGenerator(rescale=1. / 255)
validation_image_generator = tf.keras.preprocessing.image.ImageDataGenerator(rescale=1. / 255)

#定义了用于训练和验证图像的生成器之后，flow_from_directory方法从磁盘加载图像，应用缩放，并将图像大小调整到所需的尺寸。
# After defining generators for training and validation images, the flow_from_directory method loads the images from disk, applies rescaling, and resize the images to the desired dimensions。
train_data_gen = train_image_generator.flow_from_directory(batch_size=batch_size,
                                                           directory=train_dir,
                                                           shuffle=True,
                                                           target_size=(IMG_HEIGHT, IMG_WIDTH),
                                                           class_mode='binary')

val_data_gen = validation_image_generator.flow_from_directory(batch_size=batch_size,
                                                              directory=validation_dir,
                                                              target_size=(IMG_HEIGHT, IMG_WIDTH),
                                                              class_mode='binary')

#从数据集返回一个批处理
# next function: returns a batch from the dataset.
#返回值:形式为(x_train, y_train)，其中x_train为训练特性，y_train为其标签。丢弃标签，只显示训练图像。
# Return value: of the form (x_train, y_train), where x_train is the training feature and y_train is its label. Discard labels and show only training images.
sample_training_images, _ = next(train_data_gen)
test_images,label_images = next(val_data_gen)

#这个函数将图像绘制到一个由1行5列组成的网格中，图像放置在每一列中 
# This function draws the image into a grid of 1 row and 5 columns, with the image placed in each column.
#按下‘q’键即可关闭该窗口，继续运行
#Press the 'Q' key to close the window and continue running
def plotImages(images_arr):
    fig, axes = plt.subplots(1, 5, figsize=(25, 25))
    axes = axes.flatten()
    for img, ax in zip(images_arr, axes):
        ax.imshow(img)
        ax.axis('off')
    plt.tight_layout()
    plt.show()
plotImages(sample_training_images[:5])

#定义一个神经网络结构
#Define a neural network structure
model = tf.keras.models.Sequential([
    tf.keras.layers.Conv2D(16, 3, padding='same', activation='relu', input_shape=(IMG_HEIGHT, IMG_WIDTH, 3)),
    tf.keras.layers.MaxPooling2D(),
    tf.keras.layers.Conv2D(32, 3, padding='same', activation='relu'),
    tf.keras.layers.MaxPooling2D(),
    tf.keras.layers.Conv2D(64, 3, padding='same', activation='relu'),
    tf.keras.layers.MaxPooling2D(),
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(512, activation='relu'),
    tf.keras.layers.Dense(1)
])

#配置训练神经网络的方法
#Configure the method of training neural network
model.compile(optimizer='adam',
              loss=tf.keras.losses.BinaryCrossentropy(from_logits=True),
              metrics=['accuracy'])

#执行训练过程
## Implement the training process
result= model.fit(
    train_data_gen,
    steps_per_epoch = 10,
    epochs = 10
)
print(str(result.history))

test_loss,test_acc = model.evaluate(test_images,label_images,verbose=1)
print("test_acc:",test_acc)
print("test_loss:",test_loss)
#提取参数，结果可视化
#Extract parameters and visualize results
#按下‘q’键即可关闭该窗口，继续运行
#Press the 'Q' key to close the window and continue running
acc = result.history['accuracy']
loss = result.history['loss']
epochs_range = range(epochs)
plt.figure(figsize=(8, 8))
plt.subplot(1, 2, 1)
plt.plot(epochs_range, acc, label='Training Accuracy')
plt.legend(loc='lower right')
plt.title('Training Accuracy')
plt.subplot(1, 2, 2)
plt.plot(epochs_range, loss, label='Training Loss')
plt.legend(loc='upper right')
plt.title('Training Loss')
plt.show()

#打印网络结构和参数统计
#Print network structure and parameter statistics
model.summary()
