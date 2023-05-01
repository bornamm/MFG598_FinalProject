import rclpy
from rclpy.node import Node
from robdep_interfaces.msg import BlobList, BlobData

class BlobPrinter(Node):
    def __init__(self):
        super().__init__('blob_printer')
        # Define node for subcribing ti blob list and 
        self.subscription = self.create_subscription(BlobList,'/blobs_list',self.listener_callback,1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        for i in range(len(msg.data)):
            blob_data = msg.data[i]
            self.get_logger().info('X : "%s"' % blob_data.x_point)
            self.get_logger().info('Y : "%s"' % blob_data.y_point)
            self.get_logger().info('Size : "%s"' % blob_data.size)
            self.get_logger().info('Color : "%s"' % blob_data.color)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = BlobPrinter()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
