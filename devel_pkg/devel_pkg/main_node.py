import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image


class ImageParser(Node):

    def __init__(self):
        super().__init__('image_parser')
        self.subscription = self.create_subscription(Image,'/zenith_camera/image_raw',self.listener_callback,10)
        self.subscription
        self.image = image

    def listener_callback(self, img_msg):
        # Note: get encoding but for our case its rbg8
        height, width, encoding = img_msg.height, img_msg.width, img_msg.encoding
        image = np.array(img_msg.data).reshape((height,width,3))
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        self.image = image

def main(args=None):
    rclpy.init(args=args)

    image_parser = ImageParser()

    rclpy.spin(image_parser)

    image_parser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
