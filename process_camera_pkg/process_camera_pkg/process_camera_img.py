import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image


class ImageParser(Node):

    def __init__(self):
        # Create node
        super().__init__('image_parser')
        # Create subscriber
        self.subscription = self.create_subscription(Image,'/zenith_camera/image_raw',self.get_image_callback,10)
        self.subscription # Avoid warning unused variable
        # Create variable to store image
        self.image = np.zeros((240,240,3))
        # Create variable to save position of ball
        self.positions = []

    def get_image_callback(self, img_msg):
        # Note: get encoding but for our case its rbg8
        height, width, encoding = img_msg.height, img_msg.width, img_msg.encoding
        # Store image and 
        self.image = np.array(img_msg.data).reshape((height,width,3))
        self.image = cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)
        self.detect_ball_in_image()
    
    def detect_ball_in_image(self):
        cv2.imshow("Court de tennis BGR",self.image)
        cv2.waitKey(1)
        

def main(args=None):
    rclpy.init(args=args)

    image_parser = ImageParser()

    rclpy.spin(image_parser)

    cv2.destroyAllWindows()
    image_parser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
