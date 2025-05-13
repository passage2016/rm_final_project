#!/usr/bin/env python
import os
import time
import random
import ctypes
import roslib
import rospy
import smach
import smach_ros
import string
import math
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from spark_carry_object.msg import *

class GraspObject:
    def __init__(self):
        # 发布机械臂位姿
        self.pub1 = rospy.Publisher('position_write_topic', position, queue_size=10)
        # 发布机械臂吸盘
        self.pub2 = rospy.Publisher('pump_topic', status, queue_size=1)
        # 发布机械臂移动的速度
        self.pub3 = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        print(self.pub1.data_class())

        # 初始化机械臂位置
        pos = position()
        pos.x = 120
        pos.y = 0
        pos.z = 35
        self.pub1.publish(pos)
        rospy.sleep(2)  # 等待机械臂移动到位

    def move_arm(self, x, y, z):
        pos = position()
        pos.x = x
        pos.y = y
        pos.z = z
        self.pub1.publish(pos)
        rospy.sleep(2)  # 等待机械臂移动到位

    def up(self):
        pos = position()
        pos.x = 160  # 160
        pos.y = 130
        pos.z = 55  # 55
        self.pub1.publish(pos)
        rospy.sleep(2)  # 等待机械臂移动到位

    def pump_on(self):
        pmp = status()
        pmp.status = 1  # 吸盘打开
        self.pub2.publish(pmp)
        rospy.sleep(1)  # 等待吸盘动作完成

    def pump_off(self):
        pmp = status()
        pmp.status = 0  # 吸盘关闭
        self.pub2.publish(pmp)
        rospy.sleep(1)  # 等待吸盘动作完成

def main():
    try:
        rospy.init_node('cali_pos', anonymous=True)
        rospy.loginfo("Init GraspObject main")
        a = GraspObject()

        # 移动机械臂到指定位置
        a.move_arm(120, 0, 35)  # 初始位置
        a.move_arm(20, -280, 140)  # 抬起机械臂
        rospy.sleep(5)

        # 打开吸盘
        a.pump_on()
        rospy.sleep(50)

        # 移动机械臂到另一个位置
        a.move_arm(120, 0, 35)

        # # 关闭吸盘
        # a.pump_off()

    except rospy.ROSInterruptException:
        rospy.loginfo("End spark GraspObject main")

if __name__ == "__main__":
    main()
