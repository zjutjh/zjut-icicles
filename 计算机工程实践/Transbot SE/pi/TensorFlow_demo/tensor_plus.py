import tensorflow as tf
import os

a = tf.fill([2,3],6)
b = tf.fill([2,3],2)
c = a + b
print(a)
print(b)
print(c)

