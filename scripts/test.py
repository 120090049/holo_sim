# import numpy as np
# import cv2
# from cv_bridge import CvBridge

# # Load the NumPy array from the file
# data = np.load('pixels_left.npy')

# # Display the image using OpenCV
# cv2.imshow('Image', data)
# cv2.waitKey(0)  # Wait for a key press to close the window
# cv2.destroyAllWindows()  # Ensure all windows are closed

import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Initialize CvBridge
bridge = CvBridge()

# Load the NumPy array from the file
data = np.load('pixels_left.npy')

# Convert the NumPy array to a ROS Image message
ros_image = bridge.cv2_to_imgmsg(data, "bgr8")

# Convert the ROS Image message back to a cv2 (OpenCV) image
cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")

# Display the image using OpenCV
cv2.imshow('Image', cv_image)
cv2.waitKey(0)  # Wait for a key press to close the window
cv2.destroyAllWindows()  # Ensure all windows are closed
