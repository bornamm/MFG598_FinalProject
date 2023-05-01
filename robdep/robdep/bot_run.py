import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist


class BotRun(Node):

    def __init__(self):
    # Initiate the Node class's constructor and give it a name
        super().__init__('image_publisher')

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)

    # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds

    # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(0)

    # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.twist = Twist()

    def timer_callback(self):
    	
        self.twist.linear.x = 0.02
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)


def main(args=None):

    rclpy.init(args=args)
    blob_detector = BotRun()
    rclpy.spin(blob_detector)
    blob_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
