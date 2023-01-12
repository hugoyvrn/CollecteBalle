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
        # Create variable to parse HSV frame
        self.ball_low_HSV = (25,0,0)
        self.ball_high_HSV = (35,255,255)
        self.safezone_low_HSV = (10,0,0)
        self.safezone_high_HSV = (20,255,255)

    def get_image_callback(self, img_msg):
        # Note: get encoding but for our case its rbg8
        height, width, encoding = img_msg.height, img_msg.width, img_msg.encoding
        # Store image and convert it in BGR
        image_rgb = np.array(img_msg.data).reshape((height,width,3))
        self.image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        # Call the method to process the image
        self.detect_ball_in_image()
    
    def detect_ball_in_image(self):
        # Convert image to HSV
        image_HSV = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        # Get binary image with balls isolated
        image_ball = cv2.inRange(image_HSV, self.ball_low_HSV, self.ball_high_HSV)
        # Get binary image with safe zones isolated
        image_safezone = cv2.inRange(image_HSV, self.safezone_low_HSV, self.safezone_high_HSV)
        # Display to the user
        cv2.imshow("Balle(s)",image_ball)
        cv2.imshow("Safe zones",image_safezone)
        cv2.imshow("Image",self.image)
        cv2.waitKey(1)


def main(args=None):
    # Init
    rclpy.init(args=args)
    # Create the image parser
    image_parser = ImageParser()
    # Loop
    rclpy.spin(image_parser)
    # Kill properly the programs
    cv2.destroyAllWindows()
    image_parser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
