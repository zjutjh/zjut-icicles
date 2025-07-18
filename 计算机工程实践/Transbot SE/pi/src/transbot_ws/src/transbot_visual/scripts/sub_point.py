#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


def callback_pointcloud(data):
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    for p in gen: print(" x : %.3f  y: %.3f  z: %.3f" % (p[0], p[1], p[2]))


if __name__ == "__main__":
    rospy.init_node('sub_astra_point', anonymous=True)
    rospy.Subscriber('/camera/depth/points', PointCloud2, callback_pointcloud)
    rospy.spin()
