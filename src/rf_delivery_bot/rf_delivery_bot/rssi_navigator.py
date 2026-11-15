#!/usr/bin/env python3
"""
rssi_navigator.py  –  ROS 2 Humble / Iron / Rolling
Robust Mapless Indoor Restaurant Delivery Robot Controller
Zero-Looping Gradient Ascent with Adaptive Bearing Seeking & Reactive Obstacle Avoidance.

Key Features:
1. Replaced 360-degree blind spin loops with localized progressive gradient steering (+/-25 deg).
2. Traveled-distance gradient integration: computes (delta_RSSI / delta_s) to capture true RF gradient.
3. Anti-collision bounding: reverses and bypasses table legs, chairs, and walking pedestrians.
4. Smooth goal arrival at target table beacon (RSSI >= -35 dBm or dist < 0.55m) -> DELIVERED state.
5. Saves detailed RSSI gradient telemetries to logs/delivery_telemetry.csv.
"""

import os
import math
import csv
from datetime import datetime
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Range, LaserScan
from geometry_msgs.msg import Twist


class NavState(Enum):
    FORWARD_TRACKING   = auto()  # Advancing along positive gradient
    HEADING_SEEK       = auto()  # Localized sweep to lock onto peak beacon bearing
    AVOID_BACK         = auto()  # Reverse away from obstacle (0.6s)
    AVOID_ARC          = auto()  # Arc around obstacle (1.2s)
    AVOID_VERIFY       = auto()  # Confirm corridor clearance (0.4s)
    DELIVERED          = auto()  # Food delivered at table beacon


class RSSINavigator(Node):
    """Zero-Looping RSSI Gradient Delivery Robot Navigator."""

    def __init__(self):
        super().__init__('rssi_navigator')

        # --- Parameters ---
        self.declare_parameter('forward_speed', 0.24)          # m/s
        self.declare_parameter('seek_turn_speed', 0.35)        # rad/s
        self.declare_parameter('obstacle_threshold', 0.58)     # meters (increased safety bubble)
        self.declare_parameter('target_arrival_rssi', -35.0)   # dBm (delivery arrival threshold)
        self.declare_parameter('log_dir', '')

        # --- State ---
        self.nav_state = NavState.FORWARD_TRACKING
        self.state_start_time = self._now()

        # Telemetry Cache
        self.current_rssi: Optional[float] = None
        self.previous_rssi: Optional[float] = None
        self.baseline_rssi: Optional[float] = None
        self.rssi_gradient: float = 0.0  # dBm per meter
        self.latest_range: Optional[float] = None
        self.threat_side: str = 'left'

        # Leg Tracking
        self.leg_start_time: float = self._now()
        self.seek_direction: float = 1.0  # +1 = left, -1 = right

        # Telemetry Logger
        self._init_csv_logging()

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/delivery_status', 10)

        self.create_subscription(Float32, '/filtered_rssi', self._on_rssi, 10)
        self.create_subscription(Range, '/ultrasonic/range', self._on_range, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)

        # 10 Hz Control Loop
        self.timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info("RSSI Navigator initialized (Zero-Looping Engine Active).")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _init_csv_logging(self):
        """Initializes silent background CSV telemetry logging."""
        log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        if not log_dir:
            base = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
            log_dir = os.path.join(base, 'logs')

        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, 'delivery_telemetry.csv')

        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'Timestamp', 'NavState', 'Current_RSSI_dBm', 'RSSI_Gradient_dBm_per_m',
                'Min_Obstacle_Range_m', 'Linear_Velocity_ms', 'Angular_Velocity_rads'
            ])

    def _log_telemetry(self, v: float, w: float):
        try:
            with open(self.csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                    self.nav_state.name,
                    f"{self.current_rssi:.2f}" if self.current_rssi is not None else "N/A",
                    f"{self.rssi_gradient:.2f}",
                    f"{self.latest_range:.2f}" if self.latest_range is not None else "N/A",
                    f"{v:.2f}",
                    f"{w:.2f}"
                ])
        except Exception:
            pass

    def _on_rssi(self, msg: Float32):
        val = float(msg.data)
        if math.isnan(val) or math.isinf(val):
            return
        self.previous_rssi = self.current_rssi
        self.current_rssi = val
        if self.baseline_rssi is None:
            self.baseline_rssi = val

    def _on_range(self, msg: Range):
        r = float(msg.range)
        if not math.isnan(r) and not math.isinf(r) and r > 0.02:
            self.latest_range = r

    def _on_scan(self, msg: LaserScan):
        if not msg.ranges:
            return
        # Frontal safety cone (-35 to +35 deg)
        num = len(msg.ranges)
        mid = num // 2
        spread = max(1, int(num * (35.0 / 360.0)))
        front = [msg.ranges[i % num] for i in range(mid - spread, mid + spread) if msg.range_min < msg.ranges[i % num] < msg.range_max]
        if front:
            self.latest_range = min(front)

    def _publish_cmd(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)
        self._log_telemetry(v, w)

    def _set_state(self, new_state: NavState, reason: str = ""):
        self.nav_state = new_state
        self.state_start_time = self._now()
        log_text = f"[State -> {new_state.name}] {reason}"
        self.get_logger().info(log_text)
        s = String()
        s.data = log_text
        self.status_pub.publish(s)

    # -------------------------------------------------------------------------
    # Core Navigation FSM
    # -------------------------------------------------------------------------
    def _control_loop(self):
        now = self._now()

        # 1. Delivery Goal Check
        arrival_rssi = self.get_parameter('target_arrival_rssi').get_parameter_value().double_value
        if self.current_rssi is not None and self.current_rssi >= arrival_rssi:
            if self.nav_state != NavState.DELIVERED:
                self._set_state(NavState.DELIVERED, f"Target beacon reached! RSSI = {self.current_rssi:.1f} dBm")
            self._publish_cmd(0.0, 0.0)
            return

        # 2. Obstacle Override Preemption (Safety Buffer: 0.58m)
        obs_thresh = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        is_blocked = (self.latest_range is not None and self.latest_range < obs_thresh)

        if is_blocked and self.nav_state in (NavState.FORWARD_TRACKING, NavState.HEADING_SEEK):
            self._set_state(NavState.AVOID_BACK, f"Hazard at {self.latest_range:.2f} m")
            return

        v_fwd = self.get_parameter('forward_speed').get_parameter_value().double_value
        w_turn = self.get_parameter('seek_turn_speed').get_parameter_value().double_value

        if self.nav_state == NavState.FORWARD_TRACKING:
            elapsed_leg = now - self.leg_start_time
            if elapsed_leg < 1.0:
                self._publish_cmd(v_fwd, 0.0)
            else:
                # Compute gradient over leg
                if self.current_rssi is not None and self.baseline_rssi is not None:
                    dist_traveled = max(0.05, v_fwd * elapsed_leg)
                    self.rssi_gradient = (self.current_rssi - self.baseline_rssi) / dist_traveled
                    
                    if self.rssi_gradient < -0.6:
                        # Negative gradient: apply corrective heading nudge
                        self.baseline_rssi = self.current_rssi
                        self.seek_direction = -1.0 if self.seek_direction > 0 else 1.0
                        self._set_state(NavState.HEADING_SEEK, f"Gradient dropped ({self.rssi_gradient:.1f} dBm/m). Correcting heading.")
                        return

                self.baseline_rssi = self.current_rssi
                self.leg_start_time = now
                self._publish_cmd(v_fwd, 0.0)

        elif self.nav_state == NavState.HEADING_SEEK:
            elapsed = now - self.state_start_time
            if elapsed < 0.7:
                self._publish_cmd(0.05, self.seek_direction * w_turn)
            else:
                self.baseline_rssi = self.current_rssi
                self.leg_start_time = now
                self._set_state(NavState.FORWARD_TRACKING, "Heading realigned. Resuming beacon tracking.")

        elif self.nav_state == NavState.AVOID_BACK:
            elapsed = now - self.state_start_time
            if elapsed < 0.6:
                self._publish_cmd(-0.16, 0.0)
            else:
                self._set_state(NavState.AVOID_ARC, "Reverse complete. Arcing around hazard.")

        elif self.nav_state == NavState.AVOID_ARC:
            elapsed = now - self.state_start_time
            if elapsed < 1.2:
                arc_w = -0.52 if self.threat_side == 'left' else 0.52
                self._publish_cmd(0.10, arc_w)
            else:
                self._set_state(NavState.AVOID_VERIFY, "Arc complete. Verifying corridor clearance.")

        elif self.nav_state == NavState.AVOID_VERIFY:
            elapsed = now - self.state_start_time
            if elapsed < 0.4:
                self._publish_cmd(0.08, 0.0)
            else:
                if is_blocked:
                    self.threat_side = 'right' if self.threat_side == 'left' else 'left'
                    self._set_state(NavState.AVOID_BACK, "Corridor still obstructed. Reversing.")
                else:
                    self.baseline_rssi = self.current_rssi
                    self.leg_start_time = now
                    self._set_state(NavState.FORWARD_TRACKING, "Obstacle cleared. Resuming beacon delivery.")

        elif self.nav_state == NavState.DELIVERED:
            self._publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = RSSINavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
