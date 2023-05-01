import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class GetImages(Node):
    def __init__(self):
        super().__init__('subscriber')

        self.subscription = self.create_subscription(Image, '/color/preview/image', self.listener_callback, 1)
        self.br = CvBridge()
        self.num = 0

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')

        img = self.br.imgmsg_to_cv2(data)
        self.num += 1
        time.sleep(1)
        cv2.imwrite('/home/ubuntu/image_calib/images' + str(self.num) + '.png', img)


def main(args=None):

    rclpy.init(args=args)
    image_subscriber = GetImages()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
