from holoocean.lcm import RGBCamera
import lcm
import cv2
import numpy as np


# 假设你已经有了适当的 LCM 初始化和订阅逻辑

def camera_data_handler(channel, data):
    msg = RGBCamera.decode(data)

    img_array = np.array(msg.image, dtype=np.uint8)

    # 重新塑形数组以匹配图像的高度和宽度
    img_array = img_array.reshape((msg.height, msg.width, 4))

    # 如果只需要 RGB 通道，可以删除 alpha 通道
    img_array = img_array[:, :, :3]

    # 将 RGB 转换为 OpenCV 的 BGR 格式
    img_array = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)

    # 使用 OpenCV 显示图像
    cv2.imshow('Received Image', img_array)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()

# 创建 LCM 实例
lc = lcm.LCM()

# 订阅频道，假设频道名为 CAMERA_L
subscription = lc.subscribe("CAMERA_R", camera_data_handler)

try:
    print("Subscribing, waiting for data...")
    while True:
        lc.handle()
except KeyboardInterrupt:
    print("Unsubscribed and exiting...")
    lc.unsubscribe(subscription)

