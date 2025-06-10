# FireBot-Dashboard
A real-time monitoring dashboard for a firefighting robot and drone patrol system, built using ROS2 and NiceGUI. This project simulates and visualizes environmental and robot status data, making it easier to monitor critical conditions remotely.
## Features
- Map with heatmap overlay for temperature zones
- Robot & drone pose tracking
- Live camera & thermal feed switching
- Gas concentration level with status color
- Battery, extinguishing fluid level, and patrol time
- System status display
## Tech Stack
- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- [NiceGUI](https://nicegui.io/)
- WSL2 + Ubuntu 22.04(recommended)
- Python 3.10+
## Getting Started
1. Clone the repository
```
git clone https://github.com/<your-username>/FireBot-Dashboard.git
cd FireBot-Dashboard
```
2. Set up ROS2 workspace
```
cd ros2_ws
colcon build
source install/setup.bash
```
3. Install Python dependencies
```
pip install -r requirements.txt
```
4. Run the simulation
```
ros2 run fire_sim sim_node
```
5. Run the dashboard
   In another terminal:
```
cd dashboard
python3 dashboard.py
```
Then open http://localhost:8080 in your browser.
## Simulated Topics
| Topic                 | Type                       | Description                   |
| --------------------- | -------------------------- | ----------------------------- |
| `/robot_pose`         | `geometry_msgs/Pose`       | Robot's current position      |
| `/drone_pose`         | `geometry_msgs/Pose`       | Drone’s current position      |
| `/heatmap`            | `sensor_msgs/Image`        | Heat distribution             |
| `/camera`             | `sensor_msgs/Image`        | Regular camera feed           |
| `/thermal_camera`     | `sensor_msgs/Image`        | Thermal camera feed           |
| `/gas_concentration`  | `std_msgs/Float32`         | Detected gas levels           |
| `/battery`            | `sensor_msgs/BatteryState` | Battery info                  |
| `/extinguisher_level` | `std_msgs/Float32`         | Extinguishing substance level |
| `/time_remaining`     | `std_msgs/Float32`         | Estimated patrol time left    |
| `/status`             | `std_msgs/String`          | System status text            |

  
