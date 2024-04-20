#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        
        self.bridge = CvBridge()
        self.cv_image_left = None
        self.cv_image_right = None

        # 设置图像订阅
        self.sub_left = rospy.Subscriber('camera/left', Image, self.callback_left)
        self.sub_right = rospy.Subscriber('camera/right', Image, self.callback_right)

    def callback_left(self, data):
        try:
            self.cv_image_left = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridge.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def callback_right(self, data):
        try:
            self.cv_image_right = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridge.CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def show_images(self):
        if self.cv_image_left is not None and self.cv_image_right is not None:
            # 创建一个窗口来显示两个图像
            both_images = np.hstack((self.cv_image_left, self.cv_image_right))
            cv2.imshow("Camera Outputs", both_images)
            cv2.waitKey(3)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.show_images()
            rate.sleep()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    img_subscriber = ImageSubscriber()
    img_subscriber.run()
