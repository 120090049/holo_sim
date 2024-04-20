#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.bridge = CvBridge()
        rospy.init_node('image_subscriber', anonymous=True)
        self.subscriber = rospy.Subscriber(self.topic_name, Image, self.callback)
    
    def callback(self, msg):
        # Convert the ROS image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Display the converted image
        print("\033[H\033[J", end="")
        print(msg.header.stamp)
        cv2.imshow("Sonar Image Subscribe", cv_image)
        cv2.waitKey(3)  # Small delay or the image window will not be responsive

    def run(self):
        # Simply keep the node alive until it is killed
        rospy.spin()

if __name__ == '__main__':
    img_subscriber = ImageSubscriber('sonar_image_data')
    img_subscriber.run()
