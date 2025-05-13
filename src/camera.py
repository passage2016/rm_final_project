#!/usr/bin/env python
import rospy
import cv2
import time
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def save_image(data):
    try:
        cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    save_path = f'/home/spark/final/images/saved_image{sys.argv[1]}.jpg'
    # 保存图像
    cv2.imwrite(save_path, cv_image1)
    print("图像已保存到：", save_path)
    raise()

if __name__ == "__main__":
    rospy.init_node('image_converter', anonymous=True)
    print("===============11111======")
    sub1 = rospy.Subscriber("/camera/rgb/image_raw", Image, save_image)
    print("=====================")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()