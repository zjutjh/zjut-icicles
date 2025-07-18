# -*- coding: utf-8 -*-

import tensorflow.compat.v1 as tf

tf.disable_v2_behavior()

import numpy as np

import matplotlib.pyplot as plt

 

x_data = np.linspace(-0.5, 0.5, 200)[:, np.newaxis]

noise = np.random.normal(0, 0.02, x_data.shape)

y_data = np.square(x_data) + noise

 

x = tf.placeholder(tf.float32, [None, 1])

y = tf.placeholder(tf.float32, [None, 1])

 

# 输入层一个神经元，输出层一个神经元，中间10个

# 第一层

Weights_L1 = tf.Variable(tf.random.normal([1, 10]))

Biases_L1 = tf.Variable(tf.zeros([1, 10]))

Wx_plus_b_L1 = tf.matmul(x, Weights_L1) + Biases_L1

L1 = tf.nn.tanh(Wx_plus_b_L1)

 

# 第二层

Weights_L2 = tf.Variable(tf.random.normal([10, 1]))

Biases_L2 = tf.Variable(tf.zeros([1, 1]))

Wx_plus_b_L2 = tf.matmul(L1, Weights_L2) + Biases_L2

pred = tf.nn.tanh(Wx_plus_b_L2)

 

# 损失函数

loss = tf.reduce_mean(tf.square(y - pred))

 

# 训练

train = tf.train.GradientDescentOptimizer(0.1).minimize(loss)

 

with tf.Session() as sess:

    sess.run(tf.global_variables_initializer())

    for i in range(2000):

        sess.run(train, feed_dict={x: x_data, y: y_data})

        print("第{0}次，loss = {1}".format(i, sess.run(loss,feed_dict={x: x_data, y: y_data})))

    pred_vaule = sess.run(pred, feed_dict={x: x_data})

    plt.figure()

    plt.scatter(x_data, y_data)

    plt.plot(x_data, pred_vaule, 'r-', lw=5)

plt.show()
