#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt

global saved
saved = 0

def scale_to_255(a, min, max, dtype=np.uint8):
    return (((a - min) / float(max - min)) * 255).astype(dtype)

#side_range设置距离原点两侧的范围
#fwd_range设置距离原点前后的范围
def point_cloud_2_birdseye(points,
                           res=0.05,
                           side_range=(-10, 10),
                           fwd_range = (-10, 10),
                           height_range=(-2., 2.),
                           ):
	#创建一个过滤器，只保留指定区域的点
    x_points = points[:, 0]
    y_points = points[:, 1]
    z_points = points[:, 2]
    
    f_filt = np.logical_and((x_points > fwd_range[0]), (x_points < fwd_range[1]))
    s_filt = np.logical_and((y_points > -side_range[1]), (y_points < -side_range[0]))
    filter = np.logical_and(f_filt, s_filt)
    indices = np.argwhere(filter).flatten()

    x_points = x_points[indices]
    y_points = y_points[indices]
    z_points = z_points[indices]

    x_img = (-y_points / res).astype(np.int32)
    y_img = (-x_points / res).astype(np.int32)

    x_img -= int(np.floor(side_range[0] / res))
    y_img += int(np.ceil(fwd_range[1] / res))

    pixel_values = np.clip(a=z_points,
                           a_min=height_range[0],
                           a_max=height_range[1])

    pixel_values = scale_to_255(pixel_values,
                                    min=height_range[0],
                                    max=height_range[1])

    x_max = 1 + int((side_range[1] - side_range[0]) / res)
    y_max = 1 + int((fwd_range[1] - fwd_range[0]) / res)
    im = np.zeros([y_max, x_max], dtype=np.uint8)
    im[y_img, x_img] = pixel_values
    return im

def callback(lidar):
    global saved
    if saved == 0:
        lidar = pc2.read_points(lidar)
        points = np.array(list(lidar))
        im = point_cloud_2_birdseye(points)
     	plt.imshow(im, cmap="nipy_spectral", vmin=0, vmax=255)
        plt.show()
       

def cloud_subscribe():
    rospy.init_node('cloud_subscribe_node')
    rospy.Subscriber("pandar_points", PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    cloud_subscribe()

