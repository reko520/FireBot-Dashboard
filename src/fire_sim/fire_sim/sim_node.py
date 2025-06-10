import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import cv2
import random
import time


class FireSimNode(Node):
    def __init__(self):
        super().__init__('fire_sim_node')
        self.bridge = CvBridge()

        # Publishers
        self.robot_pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.drone_pose_pub = self.create_publisher(PoseStamped, '/drone_pose', 10)
        self.heatmap_pub = self.create_publisher(Float32, '/heatmap', 10)  # simplified to 1 float for now
        self.camera_pub = self.create_publisher(Image, '/camera', 10)
        self.thermal_pub = self.create_publisher(Image, '/thermal_camera', 10)
        self.status_pub = self.create_publisher(String, '/status', 10)
        self.gas_pub = self.create_publisher(Float32, '/gas_concentration', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self.extinguisher_pub = self.create_publisher(Float32, '/extinguisher_level', 10)
        self.time_remaining_pub = self.create_publisher(Float32, '/time_remaining', 10)

        self.timer = self.create_timer(1, self.publish_data)

    def publish_data(self):
        now = self.get_clock().now().to_msg()

        # Robot Pose
        robot_pose = PoseStamped()
        robot_pose.header.stamp = now
        robot_pose.header.frame_id = "map"
        robot_pose.pose.position.x = random.uniform(0.0, 10.0)
        robot_pose.pose.position.y = random.uniform(0.0, 10.0)
        self.robot_pose_pub.publish(robot_pose)

        # Drone Pose
        drone_pose = PoseStamped()
        drone_pose.header.stamp = now
        drone_pose.header.frame_id = "map"
        drone_pose.pose.position.x = random.uniform(0.0, 10.0)
        drone_pose.pose.position.y = random.uniform(0.0, 10.0)
        drone_pose.pose.position.z = 5.0
        self.drone_pose_pub.publish(drone_pose)

        # Heatmap (simulated as temperature intensity)
        self.heatmap_pub.publish(Float32(data=random.uniform(20.0, 100.0)))

        # Status
        status = random.choice(["All Good", "Warning", "Alarm"])
        self.status_pub.publish(String(data=status))

        # Gas concentration
        gas_ppm = random.uniform(50, 300)
        self.gas_pub.publish(Float32(data=gas_ppm))

        # Battery level
        battery_msg = BatteryState()
        battery_msg.percentage = random.uniform(0.3, 1.0)
        self.battery_pub.publish(battery_msg)

        # Extinguisher level
        extinguisher = random.uniform(0.0, 1.0)
        self.extinguisher_pub.publish(Float32(data=extinguisher))

        # Time remaining
        time_left = random.uniform(5.0, 30.0)
        self.time_remaining_pub.publish(Float32(data=time_left))

        # Dummy camera and thermal images
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        thermal = np.zeros((240, 320), dtype=np.uint8)

        img_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        thermal_msg = self.bridge.cv2_to_imgmsg(thermal, encoding='mono8')

        self.camera_pub.publish(img_msg)
        self.thermal_pub.publish(thermal_msg)

        self.get_logger().info(f'Published mock data @ {time.time():.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = FireSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
