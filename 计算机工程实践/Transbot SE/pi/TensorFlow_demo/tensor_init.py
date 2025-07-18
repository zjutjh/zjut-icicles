import tensorflow as tf
import os

v = tf.Variable([[1,2],[3,4]])
print(v)

v = tf.constant([[1,2],[3,4]])
print(v)

v = tf.zeros([2,2])
print(v)

v = tf.fill([2,3],8)
print(v)

v = tf.linspace(1.0,10.0,5,name="linspace")
print(v)

c = tf.ones_like(v)
print(c)
