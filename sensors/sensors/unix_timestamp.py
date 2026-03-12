import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time
from threading import Thread
class UnixTimestampPublisher(Node):

    """ROS2 node that publishes Unix timestamps for dataset synchronization."""
    def __init__(self):
        super().__init__('unix_timestamp_publisher')
        self.publisher_ = self.create_publisher(Float64, '/unix_timestamp', 10)
        self.timer = self.create_timer(0.01, self.publish_timestamp)  # 100 Hz
        self.get_logger().info('Unix timestamp publisher started')

    def publish_timestamp(self):
        msg = Float64()
        msg.data = time.time()  # Unix timestamp in seconds
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    unix_timestamp = UnixTimestampPublisher()

    rclpy.spin(unix_timestamp)

    unix_timestamp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()