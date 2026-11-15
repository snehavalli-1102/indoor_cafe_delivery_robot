#!/usr/bin/env python3
"""
rssi_filter_node.py  –  ROS 2 Humble / Iron / Rolling
High-fidelity signal filtering engine for RF beacon RSSI telemetries.
Features:
- Exponential Moving Average (EMA) and Boxcar Moving-Average algorithms
- Outlier rejection and NaN/Inf guardrails
- Autonomous simulation engine for hardware-in-the-loop and software verification
- Stale-data watchdog re-transmitter
"""

import math
from collections import deque
from typing import Optional, Deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool


class RSSIFilterNode(Node):
    """Signal conditioning node for raw wireless RSSI signals."""

    _WINDOW_SIZE   = 10
    _EMA_ALPHA     = 0.30
    _TIMEOUT_SEC   = 2.0        # Seconds before flagging signal timeout
    _SIM_AMPLITUDE = 12.0       # Signal dynamic swing in dBm
    _SIM_CENTRE    = -55.0      # Baseline beacon strength in dBm
    _SIM_FREQ_HZ   = 0.15       # Beacon gradient modulation frequency

    def __init__(self):
        super().__init__('rssi_filter')

        # --- Parameters ---
        self.declare_parameter('filter_type', 'ema')   # 'ema' or 'moving_avg'
        self.declare_parameter('sim_mode', False)      # True generates synthetic beacon
        self.declare_parameter('ema_alpha', 0.30)
        self.declare_parameter('window_size', 10)

        self._filter_type: str = (
            self.get_parameter('filter_type').get_parameter_value().string_value
        )
        self._sim_mode: bool = (
            self.get_parameter('sim_mode').get_parameter_value().bool_value
        )
        self._alpha: float = (
            self.get_parameter('ema_alpha').get_parameter_value().double_value
        )
        self._win_sz: int = int(
            self.get_parameter('window_size').get_parameter_value().integer_value
        )

        # --- State Buffers ---
        self._window: Deque[float] = deque(maxlen=self._win_sz)
        self._ema_value: Optional[float] = None
        self._last_published: Optional[float] = None
        self._last_rx_time: float = self._now()
        self._sample_count: int = 0

        # --- Communication Interfaces ---
        self._pub = self.create_publisher(Float32, '/filtered_rssi', 10)
        self._raw_sub = self.create_subscription(Float32, '/raw_rssi', self._on_raw_rssi, 10)
        self._reset_sub = self.create_subscription(Bool, '/reset_filter', self._on_reset, 10)

        if self._sim_mode:
            self._sim_start = self._now()
            # 10 Hz simulation clock
            self.create_timer(0.1, self._sim_generator_tick)
            self.get_logger().info(
                f"[Sim Engine Active] Synthetic beacon generator running: "
                f"Centre={self._SIM_CENTRE} dBm, Swing=+/-{self._SIM_AMPLITUDE} dBm, Freq={self._SIM_FREQ_HZ} Hz"
            )
        else:
            # 5 Hz watchdog checker
            self.create_timer(0.2, self._watchdog_check)

        self.get_logger().info(
            f"[rssi_filter] Node Initialized. Algorithm: '{self._filter_type.upper()}' | SimMode: {self._sim_mode}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_reset(self, msg: Bool):
        if msg.data:
            self._window.clear()
            self._ema_value = None
            self._last_published = None
            self.get_logger().info("[rssi_filter] State buffers purged via reset command.")

    def _apply_filter(self, raw_dbm: float) -> float:
        """Applies configured filter pipeline with outlier rejection."""
        # Hard limits on valid RF dBm range
        if raw_dbm > 0.0 or raw_dbm < -120.0:
            if self._last_published is not None:
                return self._last_published
            return -80.0

        if self._filter_type == 'moving_avg':
            self._window.append(raw_dbm)
            return sum(self._window) / float(len(self._window))

        # Default: Exponential Moving Average (EMA)
        if self._ema_value is None:
            self._ema_value = raw_dbm
        else:
            self._ema_value = (self._alpha * raw_dbm) + ((1.0 - self._alpha) * self._ema_value)
        return self._ema_value

    def _publish_filtered(self, value: float):
        msg = Float32()
        msg.data = float(value)
        self._pub.publish(msg)
        self._last_published = value
        self._sample_count += 1

    def _on_raw_rssi(self, msg: Float32):
        val = float(msg.data)
        if math.isnan(val) or math.isinf(val):
            return

        self._last_rx_time = self._now()
        filtered = self._apply_filter(val)
        self._publish_filtered(filtered)

    def _watchdog_check(self):
        elapsed = self._now() - self._last_rx_time
        if elapsed > self._TIMEOUT_SEC:
            if self._last_published is not None:
                self.get_logger().warn(
                    f"[Watchdog Alert] No raw beacon data for {elapsed:.1f} s. Re-broadcasting: {self._last_published:.1f} dBm"
                )
                self._publish_filtered(self._last_published)
            else:
                self.get_logger().warn("[Watchdog] Awaiting initial beacon telemetry from radio receiver.")

    def _sim_generator_tick(self):
        t = self._now() - self._sim_start
        # Simulated log-distance signal variation with Gaussian-like jitter
        base = self._SIM_CENTRE + (self._SIM_AMPLITUDE * math.sin(2.0 * math.pi * self._SIM_FREQ_HZ * t))
        noise = 1.2 * math.cos(4.7 * t)
        simulated_raw = base + noise
        filtered = self._apply_filter(simulated_raw)
        self._publish_filtered(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = RSSIFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
