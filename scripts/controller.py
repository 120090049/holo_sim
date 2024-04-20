import os
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import numpy as np
from pynput import keyboard


msg = """
%%%%%%%%%%%%%%%%%%%%%%%
action_control:
%%%%%%%%%%%%%%%%%%%%%%%
        e    
   s    d    f
e and d : pitch control
s and f : roll control
---------------------------
        i    
   j    k    l
i and k : throttle control
j and l : yaw control
---------------------------
"""

print(msg)

class Controller:
    def __init__(self, robot_index=0, val=10.0):
        os.system('stty -echo')
        
        self.robot_index = robot_index
        self.val = val
        self.pressed_keys = set()
        self.cap = False
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()

        
        # 初始化 full_command，长度为 9（一个索引 + 8个控制指令）
        self.full_command = np.zeros(9)
        self.full_command[0] = robot_index  # 设置机器人索引
        
        # ROS Publisher setup
        node_name = 'command_publisher_' + str(robot_index)
        topic_name='robot_commands_' + str(robot_index)
        rospy.init_node(node_name)
        self.publisher = rospy.Publisher(topic_name, Float32MultiArray, queue_size=10)

    def on_press(self, key):
        if hasattr(key, 'char') and key.char:
            self.pressed_keys.add(key.char)

    def on_release(self, key):
        if hasattr(key, 'char') and key.char and key.char in self.pressed_keys:
            self.pressed_keys.remove(key.char)

    def parse_keys(self):
        # 仅更新控制指令部分，索引部分已在初始化时设置
        command = np.zeros(8)  # 控制指令数组
        keys = self.pressed_keys
        val = self.val

        if 'i' in keys:
            command[0:4] += val
        if 'k' in keys:
            command[0:4] -= val
        if 'j' in keys:
            command[[4,7]] += val/20
            command[[5,6]] -= val/20
        if 'l' in keys:
            command[[4,7]] -= val/20
            command[[5,6]] += val/20

        if 'w' in keys:
            command[4:8] += val
        if 's' in keys:
            command[4:8] -= val
        if 'a' in keys:
            command[[4,6]] += val
            command[[5,7]] -= val
        if 'd' in keys:
            command[[4,6]] -= val
            command[[5,7]] += val

        # 更新 full_command 数组的控制指令部分
        self.full_command[1:] = command
        return self.full_command
    
    def publish_command(self):
        if any(self.pressed_keys):  # Check if any key is pressed
            command = self.parse_keys()
        else:
            command = np.zeros(9)
            
        msg = Float32MultiArray()
        msg.data = command.tolist()
        self.publisher.publish(msg)

    def run(self):
        rate = rospy.Rate(10)  # 控制循环以每秒10次的频率运行
        try:
            while not rospy.is_shutdown():
                self.publish_command()
                rate.sleep()
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        self.listener.stop()

if __name__ == '__main__':
    controller = Controller(robot_index=0)
    controller.run()
