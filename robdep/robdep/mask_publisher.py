import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class MaskPublisher(Node):
    def __init__(self):
        super().__init__('mask_publisher')
        self.subscription = self.create_subscription(Image, '/color/preview/image', self.listener_callback, 10)
        self.subscription

        self.br = CvBridge()

        self.publisher1_ = self.create_publisher(Image, '/masked_green', 10)
        self.publisher2_ = self.create_publisher(Image, '/masked_red', 10)
        self.publisher3_ = self.create_publisher(Image, '/masked_blue', 10)
        self.lower_blue = np.array([110, 50, 50])
        self.upper_blue = np.array([130, 255, 255])
        self.lower_red_1 = np.array([0,50,50])
        self.lower_red_2 = np.array([165,50,50])
        self.upper_red_1 = np.array([15,255,255])
        self.upper_red_2 = np.array([180,255,255])
        self.lower_green = np.array([65, 50, 50])
        self.upper_green = np.array([80, 255,255])

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.mask_green = np.zeros([250,250,3],dtype=np.uint8)
        self.mask_red = np.zeros([250,250,3],dtype=np.uint8)
        self.mask_blue = np.zeros([250,250,3],dtype=np.uint8)
        self.mask_image = np.zeros([250,250,3],dtype=np.uint8)

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')

        # MASKING FRAME
        current_frame = self.br.imgmsg_to_cv2(data)
        # current_frame = cv2.filter2D(current_frame, -1, np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]]))
        # current_frame = cv2.blur(current_frame, (5,5))
        # current_frame = cv2.resize(current_frame, (300, 300), interpolation = cv2.INTER_AREA)
        hsv_current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        self.mask_blue = cv2.inRange(hsv_current_frame, self.lower_blue, self.upper_blue)
        mask_red_1 = cv2.inRange(hsv_current_frame, self.lower_red_1, self.upper_red_1)
        mask_red_2 = cv2.inRange(hsv_current_frame, self.lower_red_2, self.upper_red_2)
        self.mask_red = mask_red_1 + mask_red_2
        self.mask_green = cv2.inRange(hsv_current_frame, self.lower_green, self.upper_green)

    def timer_callback(self):
        # self.fgMask = np.asarray(self.fgMask)
        self.publisher1_.publish(self.br.cv2_to_imgmsg(self.mask_green))
        self.publisher2_.publish(self.br.cv2_to_imgmsg(self.mask_red))
        self.publisher3_.publish(self.br.cv2_to_imgmsg(self.mask_blue))

def main(args=None):
    rclpy.init(args=args)
    background_subtraction = MaskPublisher()
    rclpy.spin(background_subtraction)
    background_subtraction.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
