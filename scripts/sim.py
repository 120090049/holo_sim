import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from holo_sim.msg import SnoarArrayData
from cv_bridge import CvBridge

import holoocean
import json
import cv2



class SimulationController:
    def __init__(self, scenario_file, robot_id='auv0'):
        # Load scenario configuration
        with open(scenario_file, 'r') as file:
            self.scenario = json.load(file)
        
        self.robot_id = robot_id
        self.command = np.zeros(5)  # Adjust based on the expected command dimensions

        # Initialize the ROS node
        rospy.init_node('simulator', anonymous=True)
        self.env = holoocean.make(scenario_cfg=self.scenario)
        self.env.should_render_viewport(False)
        self.env.set_render_quality(0)
        # Subscribers and Publishers
        self.command_sub = rospy.Subscriber('robot_commands_0', Float32MultiArray, self.command_callback)
        self.sonar_pub = rospy.Publisher('sonar_array_data', SnoarArrayData, queue_size=10)
        self.img_left_pub = rospy.Publisher('camera/left', Image, queue_size=10)
        self.img_right_pub = rospy.Publisher('camera/right', Image, queue_size=10)
        
        self.bridge = CvBridge()
        self.command = np.zeros(8)
        
    def command_callback(self, msg):
        # Extract command from the received message and update the command attribute
        self.command = np.array(msg.data[1:])  # Skip the first element (robot index)
        
    def run_simulation(self):
        rate = rospy.Rate(10)  # 控制循环以每秒10次的频率运行
        with self.env:         
            while not rospy.is_shutdown():
                # Act based on the latest command received
                self.env.act(self.robot_id, self.command)
                state = self.env.tick()
        
                timestamp = rospy.Time.from_sec(state['t'])

                # if 'PoseSensor' in state:
                #     pose = state['PoseSensor']
                
                # if "LeftCamera" in state:
                #     pixels_left = state["LeftCamera"]
                #     print(pixels_left[0])
                if "LeftCamera" in state and "RightCamera" in state:
                    pixels_left = state["LeftCamera"][:, :, 0:3]
                    pixels_right = state["RightCamera"][:, :, 0:3]
                   
                    # cv2.imshow("Camera Output left", pixels_left)
                    # cv2.waitKey(1)  # Wait for a key press to close the window
                    
                    # cv2.destroyAllWindows()
                    # 使用cv_bridge将OpenCV的图像转换为ROS的图像消息
                    ros_image_left = self.bridge.cv2_to_imgmsg(pixels_left, "bgr8")
                    ros_image_left.header.stamp = timestamp
                    ros_image_right = self.bridge.cv2_to_imgmsg(pixels_right, "bgr8")
                    ros_image_right.header.stamp = timestamp
                 
                    # 发布图像
                    self.img_left_pub.publish(ros_image_left)
                    self.img_right_pub.publish(ros_image_right)
                                        
                if 'ImagingSonar' in state:
                    s = state['ImagingSonar']
                    print(s)
                    sonar_msg = SnoarArrayData()
                    arrasonar_msg_data = Float32MultiArray()
                    arrasonar_msg_data.data = s.flatten()
                    sonar_msg.data = arrasonar_msg_data
                    sonar_msg.header.stamp = timestamp

                    # Publish the sonar data
                    self.sonar_pub.publish(sonar_msg)
            rate.sleep()
        
   

if __name__ == '__main__':
    scenario_path = '../holo_world/experiment.json'
    # scenario_path = '../holo_world/simple.json'
    # scenario_path = '../holo_world/pier_harbor_sonar.json'
    sim_controller = SimulationController(scenario_path)
    sim_controller.run_simulation()

    print("Finished Simulation!")



###########-----------------------------------simple--------------------############
    # # 看池底：
    # [[-9.77920294e-01  2.08976105e-01 -7.33731722e-05 -1.25015993e+01]
    # [ 2.08976075e-01  9.77920711e-01  1.83734868e-04  2.61993694e+01]
    # [ 1.10149354e-04  1.64344907e-04 -9.99999523e-01 -2.01950817e+01]
    # [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
    # "location": [-12.0, 26.0, -19.0],
    # "rotation": [0.0, 0.0, 102.0]

    # # 看左边水管            
    # [[-5.2147245e-01  8.5326749e-01 -6.7590631e-04 -2.3839592e+01]
    # [ 8.5326773e-01  5.2147269e-01 -3.3791515e-04  4.2718018e+01]
    # [ 6.4134540e-05 -7.5294275e-04 -9.9999928e-01 -6.9303722e+00]
    # [ 0.0000000e+00  0.0000000e+00  0.0000000e+00  1.0000000e+00]]
    # "location": [-23.0, 42.0, -7.0],
    # "rotation": [0.0, 0.0, 150.0]

    # # 看右边的水管
    # "location": [15.0, 25.0, -5.0],
    # "rotation": [0.0, 0.0, 30.0]
        # # 30度对应：                
        # [[ 8.6602545e-01  4.9999988e-01  2.5229407e-05 -2.3191376e+01]
        # [ 4.9999988e-01 -8.6602485e-01  6.8461100e-06  4.2017448e+01]
        # [ 2.5272360e-05  6.6857988e-06 -9.9999940e-01 -6.6100016e+00]
        # [ 0.0000000e+00  0.0000000e+00  0.0000000e+00  1.0000000e+00]]
                
###########-----------------------------------harbor--------------------############
# # 默认
# "location": [486.0, -632.0, -12.0],

# 中间棋盘
# [[ 8.9326888e-01  4.4943595e-01  8.8313911e-03  5.2036322e+02]
#  [ 4.4930875e-01 -8.9327741e-01  1.3301491e-02 -6.8282361e+02]
#  [ 1.3867050e-02 -7.9137860e-03 -9.9987245e-01 -1.8585316e+01]
#  [ 0.0000000e+00  0.0000000e+00  0.0000000e+00  1.0000000e+00]]
# "location": [520.0, -682.0, -18.0],

# 一个船
# [[-8.3497453e-01 -5.5028611e-01  1.2712893e-03  7.2397876e+02]
#  [-5.5028743e-01  8.3497417e-01 -1.3143568e-03 -7.1861725e+02]
#  [-3.3822126e-04 -1.7970301e-03 -9.9999774e-01 -1.7360229e+01]
#  [ 0.0000000e+00  0.0000000e+00  0.0000000e+00  1.0000000e+00]]
# "location": [723.0, -718.0, -18.0],
# "rotation": [0.0, 0.0, 150.0]



