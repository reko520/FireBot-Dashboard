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
        self.frame_count = 0

        # Publishers
        self.robot_pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.drone_pose_pub = self.create_publisher(PoseStamped, '/drone_pose', 10)
        self.heatmap_pub = self.create_publisher(Float32, '/heatmap', 10)
        self.camera_pub = self.create_publisher(Image, '/camera', 10)
        self.thermal_pub = self.create_publisher(Image, '/thermal_camera', 10)
        self.status_pub = self.create_publisher(String, '/status', 10)
        self.gas_pub = self.create_publisher(Float32, '/gas_concentration', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self.extinguisher_pub = self.create_publisher(Float32, '/extinguisher_level', 10)
        self.time_remaining_pub = self.create_publisher(Float32, '/time_remaining', 10)

        # Create two timers - one for regular data, one for camera data (faster)
        self.data_timer = self.create_timer(1, self.publish_data)
        self.camera_timer = self.create_timer(1/30, self.publish_camera_data)

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

        self.get_logger().info(f'Published mock data @ {time.time():.2f}')

    def publish_camera_data(self):
        """Publish camera and thermal images at higher frequency"""
        now = self.get_clock().now().to_msg()
        
        # Create normal camera frame
        normal_frame = self.create_normal_frame()
        normal_msg = self.bridge.cv2_to_imgmsg(normal_frame, encoding="bgr8")
        normal_msg.header.stamp = now
        normal_msg.header.frame_id = "normal_camera"
        self.camera_pub.publish(normal_msg)
        
        # Create thermal camera frame
        thermal_frame = self.create_thermal_frame()
        thermal_msg = self.bridge.cv2_to_imgmsg(thermal_frame, encoding="mono8")
        thermal_msg.header.stamp = now
        thermal_msg.header.frame_id = "thermal_camera"
        self.thermal_pub.publish(thermal_msg)
        
        self.frame_count += 1

    def create_normal_frame(self):
        """Create a mock normal camera frame"""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Draw a moving rectangle
        pos = self.frame_count % 600
        cv2.rectangle(frame, (pos, 50), (pos + 40, 90), (0, 255, 0), 2)
        
        # Draw some text
        cv2.putText(frame, "Firebot Camera", (50, 400), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Add some noise to make it look more realistic
        noise = np.random.normal(0, 10, frame.shape).astype(np.uint8)
        frame = cv2.add(frame, noise)
        
        return frame

    def create_thermal_frame(self):
        """Create a mock thermal camera frame"""
        frame = np.zeros((480, 640), dtype=np.uint8)
        
        # Create a gradient from left to right
        frame[:, :] = np.linspace(0, 255, 640, dtype=np.uint8)
        
        # Add a "hot spot" that moves around
        center_x = 320 + int(200 * np.sin(self.frame_count / 20))
        center_y = 240 + int(150 * np.cos(self.frame_count / 30))
        cv2.circle(frame, (center_x, center_y), 50, 255, -1)
        
        # Draw some text
        cv2.putText(frame, "Thermal View", (50, 400), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
        
        # Apply some blur to make it look more like a thermal image
        frame = cv2.GaussianBlur(frame, (15, 15), 0)
        
        return frame

def main(args=None):
    rclpy.init(args=args)
    node = FireSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()