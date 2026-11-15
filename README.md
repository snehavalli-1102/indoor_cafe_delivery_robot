# Autonomous Mapless Indoor Café Delivery Robot (ROS 2 & Gazebo Fortress)

Autonomous indoor delivery robot utilizing **RF/RSSI Gradient Ascent Navigation** combined with reactive **Frontal LiDAR / Ultrasonic Obstacle Avoidance** in a multi-table dining restaurant environment without prior static occupancy grid maps.

---

## 🚀 Key Features
1. **Mapless Gradient Ascent:** Drives directly toward table radio beacons using Log-Distance Path Loss RSSI gradient ($\Delta RSSI / \Delta s$) with Exponential Moving Average (EMA) filtering.
2. **Obstacle Detour & Local Minima Escape:** Handles table legs, chairs, and dynamic walking pedestrians without deadlock or trailing loops.
3. **Corridor Waypoint Guidance:** Traverses central dining aisles cleanly before branching directly to the destination table.
4. **Zero-Setup 3D Digital Twin:** Standalone browser-based 3D simulation matching Gazebo Fortress physics and restaurant world geometry (`sim_3d/restaurant_3d.html`).
5. **Real-Time Telemetry & Gradient Logging:** Logs linear/angular speeds, RSSI, and spatial gradients to CSV with automated plotting scripts.

---

## 🛠️ Quick Start (Windows Standalone)
Double-click:
* `START_3D_SIMULATION.bat` — Launches the complete 3D Gazebo digital twin.
* `VIEW_TELEMETRY_PLOTS.bat` — Generates 4-panel scientific telemetry & gradient graphs.

---

## 🤖 Running in Native ROS 2 Humble & Gazebo Fortress (WSL / Ubuntu)
```bash
# 1. Build package
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 2. Launch simulation & navigation stack
ros2 launch rf_delivery_bot delivery_launch.py
```
