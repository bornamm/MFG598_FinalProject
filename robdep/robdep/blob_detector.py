import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from robdep_interfaces.msg import BlobList, BlobData

class BlobDetector(Node):
    def __init__(self):
        super().__init__('subscriber')
        # Initialize class
        # subscribe to the image node Tx from camera and Rx by Blobdetector
        self.subscription = self.create_subscription(Image, '/color/preview/image', self.listener_callback, 100) 
        self.br = CvBridge()
        self.subscription # prevent unused variable warning
        # Define color filter constants
        self.lower_blue = np.array([110, 50, 50])
        self.upper_blue = np.array([130, 255, 255])
        self.lower_red_1 = np.array([0,50,50])
        self.lower_red_2 = np.array([165,50,50])
        self.upper_red_1 = np.array([15,255,255])
        self.upper_red_2 = np.array([180,255,255])
        self.lower_green = np.array([65, 50, 50])
        self.upper_green = np.array([80, 255,255])
        self.kernel = np.ones((5,5), np.uint8)
        # Defne interface message types
        self.blobs_b = BlobData()
        self.blobs_g = BlobData()
        self.blobs_r = BlobData()
        self.blobs = BlobList()
        # create node publisher
        self.publisher_ = self.create_publisher(BlobList, 'blobs_list', 100) 
        timer_period = 1  # seconds
        # create node timer
        self.timer = self.create_timer(timer_period, self.timer_callback) 

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        # Get current frame
        current_frame = self.br.imgmsg_to_cv2(data)
        # convert form BRG to HSV
        hsv_current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        # Create mask filters
        mask_blue = cv2.inRange(hsv_current_frame, self.lower_blue, self.upper_blue)
        mask_red_1 = cv2.inRange(hsv_current_frame, self.lower_red_1, self.upper_red_1)
        mask_red_2 = cv2.inRange(hsv_current_frame, self.lower_red_2, self.upper_red_2)
        mask_red = mask_red_1 + mask_red_2
        mask_green = cv2.inRange(hsv_current_frame, self.lower_green, self.upper_green)
        # Only check for green (conserve processing)
        min_size = 100 # Ideally should be acalibration for a more robust alrgorithm
        green_cnts, _ = cv2.findContours(mask_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        green_cnts = [c for c in green_cnts if cv2.contourArea(c) > min_size]
        sorted(green_cnts, key=cv2.contourArea, reverse=True)
        # extract detail about green blob
        for c in green_cnts:
            m = cv2.moments(c)
            self.blobs_g.x_point = float(m["m10"] / m["m00"]) # get X pos
            self.blobs_g.y_point = float(m["m01"] / m["m00"]) # get y pos
            self.blobs_g.size = cv2.contourArea(c) # get the area 
            self.blobs_g.color = "GREEN"
            self.blobs.data.append(self.blobs_g) # append to output

    def timer_callback(self):
        # publish the output
        self.publisher_.publish(self.blobs) 

# Main algorithm loop
def main(args=None):
    rclpy.init(args=args)
    blob_detector = BlobDetector()
    rclpy.spin(blob_detector)
    blob_detector.destroy_node()
    rclpy.shutdown()

# Entery point to the algorithm
if __name__ == '__main__':
    main() # Main algorithm loop function call
    

