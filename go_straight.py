#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
import sys

class SmartCarTeleop:
    def __init__(self, linear=0.2, angular=0.4):
        self.walk_vel_ = linear
        self.yaw_rate_ = angular
        self.cmdvel_ = Twist()
        self.pub_ = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.linear = linear
        self.angular = angular


    def stopRobot(self):
        self.cmdvel_.linear.x = 0.0
        self.cmdvel_.angular.z = 0.0
        self.pub_.publish(self.cmdvel_)

    def moveRight(self, time=1.0):
        # 计算需要移动的时间
        duration = time / self.angular
        # 设置速度
        self.cmdvel_.linear.x = 0
        self.cmdvel_.angular.z = - 0.1
        # 发布速度指令
        self.pub_.publish(self.cmdvel_)
        # 等待机器人移动指定距离
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.pub_.publish(self.cmdvel_)
            rate.sleep()
        # 停止机器人
        self.stopRobot()

    def moveForward(self, distance=1.0):
        # 计算需要移动的时间
        duration = distance / self.linear
        print(duration)
        # 设置速度
        self.cmdvel_.linear.x = self.linear
        self.cmdvel_.angular.z = 0.0
        # 发布速度指令
        self.pub_.publish(self.cmdvel_)
        # 等待机器人移动指定距离
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.pub_.publish(self.cmdvel_)
            rate.sleep()
        # 停止机器人
        self.stopRobot()

    def move(self, x, z):
        # 计算需要移动的时间
        duration = 0.92
        print(duration)
        # 设置速度
        self.cmdvel_.linear.x = x
        self.cmdvel_.angular.z = z
        # 发布速度指令
        self.pub_.publish(self.cmdvel_)
        # 等待机器人移动指定距离
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.pub_.publish(self.cmdvel_)
            rate.sleep()
        # 停止机器人
        self.stopRobot()
        
    def liftArm(self):
        try:
            # 调用机械臂服务抬起机械臂
            self.arm_control("lift")
            rospy.loginfo("Lifting arm...")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node("teleop_node")

    teleop = SmartCarTeleop(1, 1)
    teleop.move(1, 0)
    # teleop.moveRight()
    # teleop.moveForward()
    # rospy.spin()  # 添加rospy.spin()确保节点持续运行
