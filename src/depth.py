#!/usr/bin/env python
import time
import sys

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def pointcloud_callback(point_cloud_msg):
    # 将 PointCloud2 消息转换为可迭代的点云数据
    point_generator = pc2.read_points(point_cloud_msg, skip_nans=True)
    points = []
    for point in point_generator:
        # 获取每个点的坐标 (x, y, z)
        x = point[0]
        y = point[1]
        z = point[2]
        points.append([x, y, z])
    point_cloud = np.array(points)
    print(point_cloud)
    print(point_cloud.shape)
    np.save(f"/home/spark/final/pcd/pcd{sys.argv[1]}.npy", point_cloud)
    time.sleep(10)

if __name__ == "__main__":
    # 初始化 ROS 节点
    rospy.init_node('depth_data_listener')
    
    # 订阅深度点云话题
    rospy.Subscriber('/camera/depth/points', PointCloud2, pointcloud_callback)
    
    # 保持节点运行
    rospy.spin()
