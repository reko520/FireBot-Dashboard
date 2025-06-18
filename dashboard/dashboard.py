from nicegui import ui
import plotly.graph_objects as go
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import BatteryState, Image
from cv_bridge import CvBridge
import threading
from threading import Lock
import time
import numpy as np
import cv2
import base64

# Shared data structure with thread-safe access
class SharedData:
    def __init__(self):
        self.lock = Lock()
        self.battery_level = 0
        self.extinguishing_agent = 0
        self.robot_status = "Initializing"
        self.sensor_x = 0.0  # Distance
        self.sensor_y = 0.0  # Gas level
        self.events = ["System connecting to ROS..."]
        self.temperature_data = []
        self.normal_frame = None
        self.thermal_frame = None

    def update(self, **kwargs):
        with self.lock:
            for key, value in kwargs.items():
                setattr(self, key, value)
                if key == 'robot_status':
                    self.events.append(f"Status changed to: {value}")
                elif key == 'temperature_data':
                    self.events.append(f"New temperature reading: {value[-1]}°C")
                elif key in ['normal_frame', 'thermal_frame']:
                    self.events.append(f"New {key.replace('_', ' ')} received")
                
                # Keep only last 10 events
                if len(self.events) > 10:
                    self.events.pop(0)

shared_data = SharedData()

class ROS2Subscriber(Node):
    def __init__(self):
        super().__init__('nicegui_subscriber')
        self.bridge = CvBridge()
        
        # Create subscribers
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery', self.battery_callback, 10)
        self.extinguisher_sub = self.create_subscription(
            Float32, '/extinguisher_level', self.extinguisher_callback, 10)
        self.status_sub = self.create_subscription(
            String, '/status', self.status_callback, 10)
        self.gas_sub = self.create_subscription(
            Float32, '/gas_concentration', self.gas_callback, 10)
        self.heatmap_sub = self.create_subscription(
            Float32, '/heatmap', self.heatmap_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera', self.camera_callback, 10)
        self.thermal_sub = self.create_subscription(
            Image, '/thermal_camera', self.thermal_callback, 10)
        
    def battery_callback(self, msg):
        shared_data.update(battery_level=int(msg.percentage * 100))
        
    def extinguisher_callback(self, msg):
        shared_data.update(extinguishing_agent=int(msg.data * 100))
        
    def status_callback(self, msg):
        shared_data.update(robot_status=msg.data)
        
    def gas_callback(self, msg):
        shared_data.update(sensor_y=round(msg.data, 1))
        
    def heatmap_callback(self, msg):
        with shared_data.lock:
            new_temp = round(msg.data, 1)
            temp_data = shared_data.temperature_data.copy()
            if len(temp_data) >= 20:
                temp_data.pop(0)
            temp_data.append(new_temp)
        shared_data.update(temperature_data=temp_data, sensor_x=new_temp)
    
    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            shared_data.update(normal_frame=cv_image)
        except Exception as e:
            self.get_logger().error(f"Error processing normal image: {str(e)}")
    
    def thermal_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            # Convert grayscale to "thermal" colormap for display
            thermal_colored = cv2.applyColorMap(cv_image, cv2.COLORMAP_HOT)
            shared_data.update(thermal_frame=thermal_colored)
        except Exception as e:
            self.get_logger().error(f"Error processing thermal image: {str(e)}")

def create_circular_progress(value):
    fig = go.Figure()
    fig.add_trace(go.Pie(
        values=[value, 100 - value],
        hole=0.7,
        marker_colors=['#FF5722', '#EEEEEE'],
        textinfo='none',
        hoverinfo='none',
        rotation=90,
        direction='clockwise',
        sort=False  
    ))
    fig.update_layout(
        showlegend=False,
        margin=dict(l=10, r=10, t=10, b=10),
        annotations=[dict(text=f"{value}%", x=0.5, y=0.5, font_size=16, showarrow=False)]
    )
    return fig

def create_temperature_chart(data):
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        y=data,
        mode='lines+markers',
        line=dict(color='red')
    ))
    fig.update_layout(margin=dict(l=20, r=20, t=30, b=20))
    return fig

# Start ROS2 node in a separate thread
def ros_thread():
    rclpy.init()
    subscriber = ROS2Subscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

threading.Thread(target=ros_thread, daemon=True).start()

# Main dashboard UI
with ui.column().classes('w-full max-w-6xl mx-auto p-4'):
    # Header
    with ui.row().classes('w-full items-center justify-between mb-4'):
        with ui.row().classes('items-center'):
            ui.icon('local_fire_department', color='orange', size='lg')
            ui.label('FIREBOT DASHBOARD').classes('text-xl font-bold ml-2')
        
        with ui.tabs().classes('ml-auto') as tabs:
            ui.tab('DASHBOARD', icon='dashboard')
            ui.tab('CAMERA', icon='videocam')
            ui.tab('CONTROLS', icon='settings')
            ui.tab('LOGS', icon='list')

    # Content
    with ui.tab_panels(tabs, value='DASHBOARD').classes('w-full'):
        with ui.tab_panel('DASHBOARD'):
            # First row
            with ui.grid(columns=3).classes('w-full gap-4'):
                # Battery
                with ui.card().classes('w-full'):
                    ui.label('Battery Level').classes('text-h6')
                    battery_progress = ui.linear_progress(0, show_value=False, color='#FF5722').classes('h-4')
                    battery_label = ui.label('0%').classes('text-h4 text-center')
                
                # Extinguisher
                with ui.card().classes('w-full'):
                    ui.label('EXTINGUISHER').classes('text-lg font-medium')
                    extinguisher_plot = ui.plotly(create_circular_progress(0)).classes('w-full h-40')
                
                # Status
                with ui.card().classes('w-full'):
                    ui.label('STATUS').classes('text-lg font-medium')
                    status_label = ui.label('Unknown').classes('text-2xl text-center')

            # Second row
            with ui.grid(columns=3).classes('w-full gap-4 mt-4'):
                # Distance
                with ui.card().classes('w-full'):
                    ui.label('DISTANCE').classes('text-lg font-medium')
                    distance_label = ui.label('0 m').classes('text-2xl text-center')
                
                # Gas Level
                with ui.card().classes('w-full'):
                    ui.label('GAS LEVEL').classes('text-lg font-medium')
                    gas_label = ui.label('0 ppm').classes('text-2xl text-center')
                
                # Events
                with ui.card().classes('w-full h-48'):
                    ui.label('EVENTS').classes('text-lg font-medium mb-2')
                    with ui.scroll_area().classes('w-full h-36'):
                        events_column = ui.column().classes('w-full gap-1')

            # Chart
            with ui.card().classes('w-full mt-4'):
                ui.label('TEMPERATURE TREND').classes('text-lg font-medium mb-2')
                temperature_chart = ui.plotly(create_temperature_chart([])).classes('w-full h-64')

        # In the tab panel for CAMERA (replace the existing one)
        with ui.tab_panel('CAMERA'):
            with ui.card().classes('w-full max-w-2xl mx-auto'):  # Added max-width constraint
                ui.label('CAMERA FEED').classes('text-lg font-medium mb-2')
                with ui.row().classes('w-full justify-center'):
                    camera_toggle = ui.toggle(
                        ['Normal View', 'Thermal View'],
                        value='Normal View'
                    ).classes('mb-2')  # Reduced margin
                    
                # Camera display with fixed aspect ratio (4:3) and max height
                with ui.row().classes('w-full justify-center'):
                    camera_display = ui.image().classes('max-w-[640px] max-h-[480px] w-full')
                    
                # Compact info panel below camera
                with ui.grid(columns=2).classes('w-full mt-2 gap-2'):
                    with ui.card().tight().classes('w-full'):
                        ui.label('Temperature').classes('text-sm font-medium px-2 pt-1')
                        camera_temp_label = ui.label('0°C').classes('text-lg text-center px-2 pb-1')
                    
                    with ui.card().tight().classes('w-full'):
                        ui.label('Frame Rate').classes('text-sm font-medium px-2 pt-1')
                        fps_label = ui.label('0 FPS').classes('text-lg text-center px-2 pb-1')

# Update function that runs periodically
def update_dashboard():
    with shared_data.lock:
        # Update battery
        battery_progress.value = shared_data.battery_level / 100
        battery_label.text = f'{shared_data.battery_level}%'
        
        # Update extinguisher
        extinguisher_plot.update_figure(create_circular_progress(shared_data.extinguishing_agent))
        
        # Update status
        status_label.text = shared_data.robot_status
        if "alarm" in shared_data.robot_status.lower():
            status_label.classes(replace='text-2xl text-red-600 text-center')
        elif "warning" in shared_data.robot_status.lower():
            status_label.classes(replace='text-2xl text-yellow-600 text-center')
        else:
            status_label.classes(replace='text-2xl text-green-600 text-center')
        
        # Update distance
        distance_label.text = f'{shared_data.sensor_x} m'
        
        # Update gas level
        gas_label.text = f'{shared_data.sensor_y} ppm'
        
        # Update events
        events_column.clear()
        with events_column:
            for event in shared_data.events:
                ui.label(event).classes('text-sm w-full')
        
        # Update temperature chart
        if shared_data.temperature_data:
            temperature_chart.update_figure(create_temperature_chart(shared_data.temperature_data))
        
        # Update camera feed if we're on the camera tab
        if tabs.value == 'CAMERA':
            if camera_toggle.value == 'Normal View' and shared_data.normal_frame is not None:
                _, img = cv2.imencode('.jpg', shared_data.normal_frame)
                camera_display.set_source(f"data:image/jpeg;base64,{base64.b64encode(img).decode()}")
                camera_temp_label.text = f"{shared_data.sensor_x}°C"
            elif camera_toggle.value == 'Thermal View' and shared_data.thermal_frame is not None:
                _, img = cv2.imencode('.jpg', shared_data.thermal_frame)
                camera_display.set_source(f"data:image/jpeg;base64,{base64.b64encode(img).decode()}")
                camera_temp_label.text = f"{shared_data.sensor_x}°C"

# Update the dashboard every 500ms
ui.timer(0.5, update_dashboard)

ui.run(title="Firebot Dashboard", port=8080)