#!/usr/bin/env python
import matplotlib
matplotlib.use('Agg')  # Use the 'Agg' backend for headless environments

import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from holo_sim.msg import SnoarArrayData

from queue import Queue
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import json
import cv2

## The main goal of this script is first change the sonar array into polar coordinate system
## (which can only be realized in matplotlib)
## Then we change it into cv2 and publish it as message
## You can use the test/sonar_image_subscriber to test and receive the published sonar image

class SonarVisualizer:
    def __init__(self, config_file, sonar_array_topic, sonar_image_topic):
        # Load the configuration from the JSON file
        with open(config_file, 'r') as file:
            scenario = json.load(file)

        config = scenario['agents'][0]['sensors'][-1]["configuration"]
        self.azi = config['Azimuth']
        self.minR = config['RangeMin']
        self.maxR = config['RangeMax']
        self.binsR = config['RangeBins']
        self.binsA = config['AzimuthBins']

        # Initialize ROS node and subscriber
        rospy.init_node('sonar_listener', anonymous=True)
        rospy.Subscriber(sonar_array_topic, SnoarArrayData, self.sonar_callback)
        
        # For image publishing
        self.image_pub = rospy.Publisher(sonar_image_topic, Image, queue_size=10)
        self.bridge = CvBridge()

        # Setup the queue for thread communication
        self.sonar_data_queue = Queue()
        self.sonar_time_queue = Queue()

        # Initialize the plot
        self.init_plot()

    def init_plot(self):
        # Setup the polar plot
        plt.ion()
        # self.fig, self.ax = plt.subplots(subplot_kw=dict(projection='polar'), figsize=(10,5))
        self.fig, self.ax = plt.subplots(subplot_kw=dict(projection='polar'), figsize=(12, 6), dpi=100)  # Increased figsize and set dpi

        self.ax.set_theta_zero_location("N")
        self.ax.set_thetamin(-self.azi/2)
        self.ax.set_thetamax(self.azi/2)
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        theta = np.linspace(-self.azi/2, self.azi/2, self.binsA+1)*np.pi/180
        SonarRange = np.linspace(self.minR, self.maxR, self.binsR+1)
        T, R = np.meshgrid(theta, SonarRange)
        z = np.zeros_like(T)
        self.plot = self.ax.pcolormesh(T, R, z, cmap='afmhot', shading='auto', vmin=0, vmax=1)
        plt.grid(False)
        plt.axis('off')
        plt.tight_layout()
        plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
    
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # Make the plot occupy the maximum space
        self.fig.subplots_adjust(left=0, right=1, top=1, bottom=0)  # Reduce padding and margins
        self.fig.tight_layout(pad=0)  # Make the layout tight

    def sonar_callback(self, msg):
        
        sonar_image_array = np.array(msg.data.data)  # be attentation! msg here is SnoarArrayData 
                                                     # msg.data is Float32MultiArray
                                                     # so we need Float32MultiArray.data here
        self.sonar_data_queue.put(sonar_image_array)
        self.sonar_time_queue.put(msg.header.stamp)

    def publish_image(self, timestamp):
        # Convert the matplotlib figure to an OpenCV image
        # Grab the buffer from the canvas, which stores the plot image
        buf = self.fig.canvas.tostring_rgb()
        
        # Convert buffer to numpy array
        img = np.frombuffer(buf, dtype=np.uint8)
        img = img.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # Convert OpenCV image to ROS message and publish
        image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        image_msg.header.stamp = timestamp
        self.image_pub.publish(image_msg)
    
    def run(self):
        rate = rospy.Rate(10)  # 控制循环以每秒10次的频率运行
        
        while not rospy.is_shutdown():
            if not self.sonar_data_queue.empty():
                sonar_image = self.sonar_data_queue.get()

                self.plot.set_array(sonar_image)
                
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
                
                timestamp = self.sonar_time_queue.get()
                self.publish_image(timestamp)
                rate.sleep()
                
                    
    
        
   
if __name__ == '__main__':
    
    # Check the visualize flag
    visualizer = SonarVisualizer('../holo_world/experiment.json', 'sonar_array_data', 'sonar_image_data')
    visualizer.run()
        

        
