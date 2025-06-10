from nicegui import ui
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from sensor_msgs.msg import BatteryState

from threading import Thread
import base64
import cv2
import numpy as np

# Initialize UI Elements
with ui.row().classes('w-full justify-between items-center'):
    ui.label('🔥 Firefighting Robot Dashboard').classes('text-2xl font-bold')
    show_thermal = ui.toggle(["Camera", "Thermal"], value="Camera")

with ui.row().classes('w-full'):
    camera_view = ui.image().classes('w-1/2')
    thermal_view = ui.image().classes('w-1/2').props('hidden')

with ui.row().classes('w-full justify-around'):
    status = ui.label("Status: Waiting...").classes('text-lg')
    gas = ui.label("Gas: --").classes('text-lg')
    battery = ui.label("Battery: --").classes('text-lg')
    extinguishing = ui.label("Extinguishing: --").classes('text-lg')
    patrol_time = ui.label("Patrol Time: --").classes('text-lg')


def on_toggle_change():
    if show_thermal.value == "Camera":
        camera_view.props('hidden', False)
        thermal_view.props('hidden', True)
    else:
        camera_view.props('hidden', True)
        thermal_view.props('hidden', False)

show_thermal.on('update', on_toggle_change)


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_gui')

        self.create_subscription(String, 'status', self.cb_status, 10)
        self.create_subscription(Float32, 'gas_concentration', self.cb_gas, 10)
        self.create_subscription(BatteryState, 'battery', self.cb_battery, 10)
        self.create_subscription(Float32, 'extinguisher_level', self.cb_extinguish, 10)
        self.create_subscription(Float32, 'time_remaining', self.cb_patrol, 10)
        self.create_subscription(Image, 'camera', self.cb_camera, 10)
        self.create_subscription(Image, 'thermal_camera', self.cb_thermal, 10)

    def cb_status(self, msg):
        status.text = f"Status: {msg.data}"

    def cb_gas(self, msg):
        val = msg.data
        level = '🟢 Good' if val < 30 else '🟡 Medium' if val < 70 else '🔴 Bad'
        gas.text = f"Gas: {val:.1f} ppm ({level})"

    def cb_battery(self, msg: BatteryState):
        self.battery_level = f"{msg.percentage * 100:.1f}%"
        battery.text = f"Battery: {self.battery_level}"

    def cb_extinguish(self, msg):
        extinguishing.text = f"Extinguishing: {msg.data:.1f}%"

    def cb_patrol(self, msg):
        patrol_time.text = f"Patrol Time: {msg.data:.1f} min"

    def cb_camera(self, msg):
        self.update_image(msg, camera_view)

    def cb_thermal(self, msg):
        self.update_image(msg, thermal_view)

    def update_image(self, msg, image_element):
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
            _, buf = cv2.imencode('.jpg', img)
            img_b64 = base64.b64encode(buf).decode('utf-8')
            image_element.set_source(f'data:image/jpeg;base64,{img_b64}')
        except Exception as e:
            self.get_logger().warn(f'Failed to display image: {e}')


def run_ros():
    rclpy.init()
    node = DashboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


Thread(target=run_ros, daemon=True).start()
ui.run(title='Firefighting Robot Dashboard', reload=False)
