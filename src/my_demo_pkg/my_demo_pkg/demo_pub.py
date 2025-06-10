import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
import time

class DemoPublisher(Node):
    def __init__(self):
        super().__init__('demo_publisher')
        self.publisher_ = self.create_publisher(Float64, 'demo_data', 10)
        self.timer = self.create_timer(1.0, self.publish_data)

    def publish_data(self):
        msg = Float64()
        msg.data = random.uniform(0.0, 100.0)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main():
    rclpy.init()
    node = DemoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
