#!/usr/bin/env python3
"""
cafe_simulator_gui.py
Standalone Desktop Simulation & Interactive Telemetry Dashboard
for the ROS 2 Mapless Indoor Café Delivery Robot.

Runs natively on Windows / Linux / macOS with ZERO external dependencies (Pure Python + Tkinter).
Strictly implements the exact RSSI Log-Distance Radio Model, Ultrasonic Raycasting,
Signal Filtering (EMA / Moving Average), and 5-Phase Obstacle Avoidance State Machine.
"""

import math
import random
import time
from collections import deque
from enum import Enum, auto
import tkinter as tk
from tkinter import ttk, messagebox


# =============================================================================
# State Machine & Mathematical Definitions
# =============================================================================
class AvoidPhase(Enum):
    CLEAR     = ("CLEAR", "#2ecc71")       # Green: Tracking beacon gradient
    BACK      = ("BACK", "#e74c3c")        # Red: Reversing away from obstacle
    ARC       = ("ARC", "#e67e22")         # Orange: Clearance arc-turn
    VERIFY    = ("VERIFY", "#f1c40f")      # Yellow: Verifying corridor clearance
    RESUME    = ("RESUME", "#3498db")      # Blue: Resyncing gradient reference
    RECOVERY  = ("RECOVERY", "#9b59b6")    # Purple: 180-deg emergency corner escape
    DELIVERED = ("DELIVERED", "#1abc9c")   # Cyan: Target table reached!


class TableTarget:
    def __init__(self, table_id: int, name: str, x: float, y: float):
        self.id = table_id
        self.name = name
        self.x = x
        self.y = y


class Obstacle:
    def __init__(self, x: float, y: float, radius: float, name: str, color: str = "#7f8c8d", dynamic: bool = False):
        self.x = x
        self.y = y
        self.radius = radius
        self.name = name
        self.color = color
        self.dynamic = dynamic
        self.vx = 0.08 if dynamic else 0.0
        self.vy = 0.04 if dynamic else 0.0
        self.x_bounds = (x - 1.2, x + 1.2)
        self.y_bounds = (y - 0.8, y + 0.8)


# =============================================================================
# Core Simulation Engine
# =============================================================================
class CafeSimulationApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("ROS 2 Mapless Indoor Café Delivery Robot — Windows Interactive Simulation")
        self.root.geometry("1240x820")
        self.root.minsize(1050, 720)
        self.root.configure(bg="#1e272e")

        # Simulation World Scale (15m x 12m café)
        self.world_w = 15.0  # -7.5 to +7.5 m
        self.world_h = 12.0  # -6.0 to +6.0 m
        self.canvas_w = 680
        self.canvas_h = 740

        # Robot Physical State (Spawn at entrance x=0, y=-4.5, facing North pi/2)
        self.spawn_x = 0.0
        self.spawn_y = -4.5
        self.spawn_theta = math.pi / 2.0

        self.robot_x = self.spawn_x
        self.robot_y = self.spawn_y
        self.robot_theta = self.spawn_theta
        self.robot_radius = 0.28   # meters
        self.v = 0.0
        self.omega = 0.0
        self.trail = deque(maxlen=150)

        # Simulation Loop Control
        self.is_running = True
        self.sim_speed = 1.0
        self.last_sim_time = time.time()
        self.start_timestamp = time.time()
        self.total_distance = 0.0
        self.manual_mode = False

        # Radio Propagation (Log-Distance Path Loss Model)
        # RSSI(d) = P0 - 10 * n * log10(d) + X_sigma
        self.P0 = -32.0         # RSSI at 1 meter distance (dBm)
        self.path_loss_n = 2.4  # Path loss exponent (indoor café multipath)
        self.sigma = 1.8        # Shadowing noise standard deviation (dB)

        # Tables & Target Beacon
        self.tables = [
            TableTarget(1, "Table 1", -4.0,  2.0),
            TableTarget(2, "Table 2", -4.0, -2.0),
            TableTarget(3, "Table 3",  0.0,  2.0),
            TableTarget(4, "Table 4",  0.0, -2.0),
            TableTarget(5, "Table 5",  4.0,  2.0),
            TableTarget(6, "Table 6",  4.0, -2.0),
        ]
        self.current_target_idx = 0
        self.beacon = self.tables[self.current_target_idx]

        # Obstacles Setup (Tables, Chairs, Kitchen Counter, Pedestrians)
        self.obstacles: list[Obstacle] = []
        self._init_world_obstacles()

        # Ultrasonic Sensing
        self.sensor_fov = math.radians(45.0)  # +/-22.5 deg cone
        self.sensor_max_range = 2.8           # meters
        self.latest_range = 999.0
        self.obstacle_threshold = 0.50        # safety threshold (m)
        self.sensor_ray_endpoint = (self.robot_x, self.robot_y)

        # Signal Filtering (EMA vs Moving Avg)
        self.filter_algorithm = "EMA"
        self.ema_alpha = 0.30
        self.raw_rssi = -60.0
        self.filtered_rssi = -60.0
        self.previous_rssi = -60.0
        self.ema_state = None
        self.ma_buffer = deque(maxlen=10)

        # Oscilloscope Chart History
        self.chart_raw_history = deque(maxlen=70)
        self.chart_filt_history = deque(maxlen=70)
        for _ in range(70):
            self.chart_raw_history.append(-65.0)
            self.chart_filt_history.append(-65.0)

        # FSM State
        self.fsm_phase = AvoidPhase.CLEAR
        self.phase_start_time = time.time()
        self.threat_side = "left"
        self.consecutive_avoids = 0
        self.avoidance_count = 0
        self.deliveries_count = 0

        # Gradient Search Sweep
        self.is_sweeping = False
        self.sweep_start_time = 0.0
        self.sweep_duration = (2.0 * math.pi) / 0.40
        self.best_sweep_rssi = -999.0
        self.sweep_locked = False

        # Build GUI Layout
        self._build_ui()
        self._bind_keys()

        # Start Continuous Animation Loop (30 FPS)
        self.root.after(33, self._sim_tick)

    # -------------------------------------------------------------------------
    # World Geometry Initialization
    # -------------------------------------------------------------------------
    def _init_world_obstacles(self):
        self.obstacles.clear()

        # 6 Tables (Radius 0.42m with chairs)
        for t in self.tables:
            self.obstacles.append(Obstacle(t.x, t.y, 0.42, f"Table {t.id}", "#8e44ad"))
            # Chairs around table
            self.obstacles.append(Obstacle(t.x, t.y + 0.65, 0.22, f"Chair {t.id}A", "#57606f"))
            self.obstacles.append(Obstacle(t.x, t.y - 0.65, 0.22, f"Chair {t.id}B", "#57606f"))

        # Kitchen Counter (Segmented as 3 circular collision zones at y=4.8)
        self.obstacles.append(Obstacle(-1.0, 4.8, 0.45, "Counter Left", "#d35400"))
        self.obstacles.append(Obstacle( 0.0, 4.8, 0.45, "Counter Center", "#d35400"))
        self.obstacles.append(Obstacle( 1.0, 4.8, 0.45, "Counter Right", "#d35400"))

        # Pedestrians (Dynamic human obstacles walking around)
        self.obstacles.append(Obstacle( 0.0,  3.4, 0.30, "Pedestrian 1 (Counter)", "#2980b9", dynamic=True))
        self.obstacles.append(Obstacle( 2.2,  0.4, 0.30, "Pedestrian 2 (Aisle)",   "#c0392b", dynamic=True))
        self.obstacles.append(Obstacle(-2.2, -1.0, 0.30, "Pedestrian 3 (Walkway)", "#27ae60", dynamic=True))

    # -------------------------------------------------------------------------
    # Coordinate Conversion: World (m) <-> Canvas (px)
    # -------------------------------------------------------------------------
    def w2c(self, wx: float, wy: float):
        # Center of canvas is (0, 0) in world
        # world_x: -7.5 to 7.5 -> canvas_x: 30 to canvas_w - 30
        margin = 35
        usable_w = self.canvas_w - (2 * margin)
        usable_h = self.canvas_h - (2 * margin)

        cx = margin + ((wx + (self.world_w / 2.0)) / self.world_w) * usable_w
        # Invert y because canvas (0,0) is top-left
        cy = margin + (((self.world_h / 2.0) - wy) / self.world_h) * usable_h
        return cx, cy

    def c2w(self, cx: float, cy: float):
        margin = 35
        usable_w = self.canvas_w - (2 * margin)
        usable_h = self.canvas_h - (2 * margin)

        wx = (((cx - margin) / usable_w) * self.world_w) - (self.world_w / 2.0)
        wy = (self.world_h / 2.0) - (((cy - margin) / usable_h) * self.world_h)
        return wx, wy

    # -------------------------------------------------------------------------
    # UI Layout & Aesthetics
    # -------------------------------------------------------------------------
    def _build_ui(self):
        # Main split container
        container = tk.Frame(self.root, bg="#1e272e")
        container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # --- LEFT: Simulation Canvas ---
        left_box = tk.Frame(container, bg="#2f3640", bd=2, relief=tk.GROOVE)
        left_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=(0, 10))

        title_lbl = tk.Label(
            left_box,
            text="☕ CAFÉ TOP-DOWN DIGITAL TWIN (15m x 12m)",
            font=("Segoe UI", 12, "bold"),
            bg="#2f3640",
            fg="#f5f6fa"
        )
        title_lbl.pack(pady=6)

        sub_lbl = tk.Label(
            left_box,
            text="💡 Click on canvas to spawn custom obstacle | Green: Beacon | Red: Detection",
            font=("Segoe UI", 9),
            bg="#2f3640",
            fg="#718093"
        )
        sub_lbl.pack(pady=(0, 4))

        self.canvas = tk.Canvas(
            left_box,
            width=self.canvas_w,
            height=self.canvas_h,
            bg="#2c3e50",
            highlightthickness=0
        )
        self.canvas.pack(padx=8, pady=8)
        self.canvas.bind("<Button-1>", self._on_canvas_click)

        # --- RIGHT: Control & Oscilloscope Panel ---
        right_box = tk.Frame(container, bg="#1e272e")
        right_box.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # 1. State Machine Banner
        self.fsm_card = tk.Frame(right_box, bg="#2ecc71", bd=3, relief=tk.RAISED)
        self.fsm_card.pack(fill=tk.X, pady=(0, 8))

        self.fsm_state_label = tk.Label(
            self.fsm_card,
            text="STATE: CLEAR — RSSI GRADIENT TRACKING",
            font=("Consolas", 14, "bold"),
            bg="#2ecc71",
            fg="#ffffff",
            pady=8
        )
        self.fsm_state_label.pack()

        # 2. Oscilloscope (Real-time RSSI Graph)
        chart_frame = tk.LabelFrame(
            right_box,
            text=" 📡 LIVE RSSI SIGNAL OSCILLOSCOPE (dBm) ",
            font=("Segoe UI", 10, "bold"),
            bg="#2f3640",
            fg="#00d2d3",
            bd=2
        )
        chart_frame.pack(fill=tk.X, pady=(0, 8))

        self.chart_canvas = tk.Canvas(chart_frame, width=460, height=130, bg="#192a56", highlightthickness=0)
        self.chart_canvas.pack(padx=6, pady=6)

        chart_legend = tk.Label(
            chart_frame,
            text="● Cyan: Raw Noisy RSSI  |  ● Lime: Filtered Signal  |  --- Red: Goal (-36 dBm)",
            font=("Segoe UI", 8),
            bg="#2f3640",
            fg="#dcdde1"
        )
        chart_legend.pack(pady=(0, 4))

        # 3. Telemetry Grid
        telemetry_frame = tk.LabelFrame(
            right_box,
            text=" 📊 REAL-TIME ROBOT TELEMETRY ",
            font=("Segoe UI", 10, "bold"),
            bg="#2f3640",
            fg="#fbc531",
            bd=2
        )
        telemetry_frame.pack(fill=tk.X, pady=(0, 8))

        self.lbl_pos = self._create_telem_row(telemetry_frame, "Coordinates (X, Y):", "0.00 m, -4.50 m")
        self.lbl_heading = self._create_telem_row(telemetry_frame, "Robot Heading (θ):", "90.0° (North)")
        self.lbl_vel = self._create_telem_row(telemetry_frame, "Velocity (v, ω):", "0.22 m/s, 0.00 rad/s")
        self.lbl_rssi = self._create_telem_row(telemetry_frame, "RSSI (Raw / Filtered):", "-65.0 dBm / -65.0 dBm")
        self.lbl_dist = self._create_telem_row(telemetry_frame, "Target Beacon Dist:", "6.50 m (Table 1)")
        self.lbl_range = self._create_telem_row(telemetry_frame, "Ultrasonic Sensor Range:", "> 2.50 m (CLEAR)")
        self.lbl_metrics = self._create_telem_row(telemetry_frame, "Delivered / Avoided:", "0 Orders / 0 Obstacles")

        # 4. Mission Control & Target Selection
        control_frame = tk.LabelFrame(
            right_box,
            text=" 🎯 MISSION DISPATCHER & CONTROLS ",
            font=("Segoe UI", 10, "bold"),
            bg="#2f3640",
            fg="#4cd137",
            bd=2
        )
        control_frame.pack(fill=tk.BOTH, expand=True)

        target_row = tk.Frame(control_frame, bg="#2f3640")
        target_row.pack(fill=tk.X, padx=10, pady=6)

        tk.Label(target_row, text="Select Target Table:", font=("Segoe UI", 9, "bold"), bg="#2f3640", fg="#f5f6fa").pack(side=tk.LEFT)
        self.target_var = tk.StringVar(value="Table 1")
        target_combo = ttk.Combobox(target_row, textvariable=self.target_var, values=[t.name for t in self.tables], state="readonly", width=12)
        target_combo.pack(side=tk.LEFT, padx=8)
        target_combo.bind("<<ComboboxSelected>>", self._on_target_changed)

        btn_dispatch = tk.Button(target_row, text="🚀 Dispatch Order", bg="#27ae60", fg="white", font=("Segoe UI", 9, "bold"), command=self._dispatch_order)
        btn_dispatch.pack(side=tk.LEFT, padx=4)

        # Filter Algorithm Selector
        filt_row = tk.Frame(control_frame, bg="#2f3640")
        filt_row.pack(fill=tk.X, padx=10, pady=4)
        tk.Label(filt_row, text="Signal Filter Algorithm:", font=("Segoe UI", 9, "bold"), bg="#2f3640", fg="#f5f6fa").pack(side=tk.LEFT)
        self.filter_var = tk.StringVar(value="EMA (alpha=0.30)")
        filt_combo = ttk.Combobox(filt_row, textvariable=self.filter_var, values=["EMA (alpha=0.30)", "Moving Average (window=10)"], state="readonly", width=24)
        filt_combo.pack(side=tk.LEFT, padx=8)
        filt_combo.bind("<<ComboboxSelected>>", self._on_filter_changed)

        # Simulation Speed & Buttons
        btn_row = tk.Frame(control_frame, bg="#2f3640")
        btn_row.pack(fill=tk.X, padx=10, pady=8)

        self.btn_pause = tk.Button(btn_row, text="⏸ Pause Sim", bg="#e67e22", fg="white", font=("Segoe UI", 9, "bold"), width=11, command=self._toggle_pause)
        self.btn_pause.pack(side=tk.LEFT, padx=3)

        btn_reset = tk.Button(btn_row, text="🔄 Reset Robot", bg="#2980b9", fg="white", font=("Segoe UI", 9, "bold"), width=11, command=self._reset_robot)
        btn_reset.pack(side=tk.LEFT, padx=3)

        btn_clear_obs = tk.Button(btn_row, text="🧹 Clear Obstacles", bg="#7f8c8d", fg="white", font=("Segoe UI", 9), width=13, command=self._clear_custom_obstacles)
        btn_clear_obs.pack(side=tk.LEFT, padx=3)

        speed_row = tk.Frame(control_frame, bg="#2f3640")
        speed_row.pack(fill=tk.X, padx=10, pady=4)
        tk.Label(speed_row, text="Simulation Speed:", font=("Segoe UI", 9), bg="#2f3640", fg="#dcdde1").pack(side=tk.LEFT)
        for spd in [1.0, 2.0, 4.0]:
            btn_spd = tk.Button(speed_row, text=f"{int(spd)}x", bg="#353b48", fg="white", width=4, command=lambda s=spd: self._set_speed(s))
            btn_spd.pack(side=tk.LEFT, padx=2)

        # Manual Mode Note
        manual_lbl = tk.Label(
            control_frame,
            text="⌨ Manual Override: Press [W/A/S/D] or [Arrow Keys] to take manual drive control | [SPACE] to toggle autonomous",
            font=("Segoe UI", 8),
            bg="#2f3640",
            fg="#00d2d3"
        )
        manual_lbl.pack(pady=6)

    def _create_telem_row(self, parent, label_text: str, default_val: str):
        row = tk.Frame(parent, bg="#2f3640")
        row.pack(fill=tk.X, padx=8, pady=2)
        lbl_k = tk.Label(row, text=label_text, font=("Segoe UI", 9), bg="#2f3640", fg="#dcdde1", width=22, anchor="w")
        lbl_k.pack(side=tk.LEFT)
        lbl_v = tk.Label(row, text=default_val, font=("Consolas", 9, "bold"), bg="#2f3640", fg="#00d2d3", anchor="w")
        lbl_v.pack(side=tk.LEFT, fill=tk.X, expand=True)
        return lbl_v

    def _bind_keys(self):
        self.root.bind("<Up>", lambda e: self._manual_drive(0.25, 0.0))
        self.root.bind("<Down>", lambda e: self._manual_drive(-0.18, 0.0))
        self.root.bind("<Left>", lambda e: self._manual_drive(0.08, 0.6))
        self.root.bind("<Right>", lambda e: self._manual_drive(0.08, -0.6))
        self.root.bind("<w>", lambda e: self._manual_drive(0.25, 0.0))
        self.root.bind("<s>", lambda e: self._manual_drive(-0.18, 0.0))
        self.root.bind("<a>", lambda e: self._manual_drive(0.08, 0.6))
        self.root.bind("<d>", lambda e: self._manual_drive(0.08, -0.6))
        self.root.bind("<space>", lambda e: self._toggle_manual())

    # -------------------------------------------------------------------------
    # Event Handlers & Control Commands
    # -------------------------------------------------------------------------
    def _manual_drive(self, linear_v: float, angular_w: float):
        self.manual_mode = True
        self.v = linear_v
        self.omega = angular_w

    def _toggle_manual(self):
        self.manual_mode = not self.manual_mode
        if not self.manual_mode:
            self.v = 0.0
            self.omega = 0.0

    def _toggle_pause(self):
        self.is_running = not self.is_running
        self.btn_pause.configure(text="▶ Resume Sim" if not self.is_running else "⏸ Pause Sim")

    def _set_speed(self, speed: float):
        self.sim_speed = speed

    def _on_target_changed(self, event=None):
        name = self.target_var.get()
        for idx, t in enumerate(self.tables):
            if t.name == name:
                self.current_target_idx = idx
                self.beacon = t
                break

    def _dispatch_order(self):
        self._on_target_changed()
        self.fsm_phase = AvoidPhase.CLEAR
        self.is_sweeping = False
        self.sweep_locked = False
        self.consecutive_avoids = 0
        self.manual_mode = False
        messagebox.showinfo("Order Dispatched", f"Delivery order dispatched to {self.beacon.name}!\nRobot initiating RSSI gradient navigation.")

    def _on_filter_changed(self, event=None):
        val = self.filter_var.get()
        if "EMA" in val:
            self.filter_algorithm = "EMA"
        else:
            self.filter_algorithm = "MA"
        self.ema_state = None
        self.ma_buffer.clear()

    def _reset_robot(self):
        self.robot_x = self.spawn_x
        self.robot_y = self.spawn_y
        self.robot_theta = self.spawn_theta
        self.v = 0.0
        self.omega = 0.0
        self.fsm_phase = AvoidPhase.CLEAR
        self.trail.clear()
        self.is_sweeping = False
        self.sweep_locked = False
        self.consecutive_avoids = 0
        self.manual_mode = False

    def _clear_custom_obstacles(self):
        # Keep static world elements, drop custom placed
        self._init_world_obstacles()

    def _on_canvas_click(self, event):
        wx, wy = self.c2w(event.x, event.y)
        # Avoid placing on top of robot or counter
        if math.hypot(wx - self.robot_x, wy - self.robot_y) < 0.6:
            return
        if wy > 4.2:
            return

        self.obstacles.append(Obstacle(wx, wy, 0.28, "Custom Obstacle", "#e74c3c"))

    # -------------------------------------------------------------------------
    # Physics & Sensor Simulation
    # -------------------------------------------------------------------------
    def _compute_rssi(self, dt: float) -> tuple[float, float]:
        """Log-Distance Path Loss Model with Shadowing."""
        dist = math.hypot(self.robot_x - self.beacon.x, self.robot_y - self.beacon.y)
        dist = max(0.2, dist)  # guard near-field singularity

        # RSSI = P0 - 10*n*log10(d) + N(0, sigma^2)
        mean_rssi = self.P0 - (10.0 * self.path_loss_n * math.log10(dist))
        noise = random.gauss(0.0, self.sigma)
        raw = mean_rssi + noise

        # Filter
        if self.filter_algorithm == "MA":
            self.ma_buffer.append(raw)
            filtered = sum(self.ma_buffer) / len(self.ma_buffer)
        else:
            # EMA
            if self.ema_state is None:
                self.ema_state = raw
            else:
                self.ema_state = (self.ema_alpha * raw) + ((1.0 - self.ema_alpha) * self.ema_state)
            filtered = self.ema_state

        return raw, filtered

    def _raycast_ultrasonic(self) -> float:
        """Simulates ultrasonic sensor cone raycasting."""
        min_dist = self.sensor_max_range
        best_endpoint = (
            self.robot_x + min_dist * math.cos(self.robot_theta),
            self.robot_y + min_dist * math.sin(self.robot_theta)
        )

        # Cast 7 rays across the +/-22.5 deg FOV
        num_rays = 7
        angles = [
            self.robot_theta - (self.sensor_fov / 2.0) + (i * (self.sensor_fov / (num_rays - 1)))
            for i in range(num_rays)
        ]

        for ray_angle in angles:
            cos_a = math.cos(ray_angle)
            sin_a = math.sin(ray_angle)

            # Check all obstacles
            for obs in self.obstacles:
                # Vector from robot to obstacle
                dx = obs.x - self.robot_x
                dy = obs.y - self.robot_y

                # Projection of obstacle center onto ray
                proj = (dx * cos_a) + (dy * sin_a)
                if proj <= 0:
                    continue

                # Perpendicular distance from obstacle center to ray
                perp_sq = (dx * dx + dy * dy) - (proj * proj)
                if perp_sq < (obs.radius * obs.radius):
                    # Collision with circle!
                    half_chord = math.sqrt(max(0.0, (obs.radius * obs.radius) - perp_sq))
                    hit_dist = proj - half_chord
                    if 0.05 < hit_dist < min_dist:
                        min_dist = hit_dist
                        best_endpoint = (
                            self.robot_x + hit_dist * cos_a,
                            self.robot_y + hit_dist * sin_a
                        )

            # Check North, South, East, West walls
            # East (x = 7.5), West (x = -7.5)
            if cos_a > 1e-4:
                d_wall = (7.5 - self.robot_x) / cos_a
                if 0 < d_wall < min_dist:
                    min_dist = d_wall
            elif cos_a < -1e-4:
                d_wall = (-7.5 - self.robot_x) / cos_a
                if 0 < d_wall < min_dist:
                    min_dist = d_wall

            # North (y = 6.0), South (y = -6.0)
            if sin_a > 1e-4:
                d_wall = (6.0 - self.robot_y) / sin_a
                if 0 < d_wall < min_dist:
                    min_dist = d_wall
            elif sin_a < -1e-4:
                d_wall = (-6.0 - self.robot_y) / sin_a
                if 0 < d_wall < min_dist:
                    min_dist = d_wall

        self.sensor_ray_endpoint = best_endpoint
        return min_dist

    # -------------------------------------------------------------------------
    # Finite State Machine Controller
    # -------------------------------------------------------------------------
    def _update_fsm(self, dt: float):
        now = time.time()

        # Goal Arrival Check: Distance < 0.45m or RSSI > -36 dBm
        dist_to_beacon = math.hypot(self.robot_x - self.beacon.x, self.robot_y - self.beacon.y)
        if dist_to_beacon < 0.50 or self.filtered_rssi >= -36.0:
            if self.fsm_phase != AvoidPhase.DELIVERED:
                self.fsm_phase = AvoidPhase.DELIVERED
                self.deliveries_count += 1
                self.v = 0.0
                self.omega = 0.0
            return

        # FSM Execution
        phase = self.fsm_phase

        if phase == AvoidPhase.CLEAR:
            # Safety Preemption Check
            if self.latest_range < self.obstacle_threshold:
                self.avoidance_count += 1
                self.fsm_phase = AvoidPhase.BACK
                self.phase_start_time = now
                self.v = -0.15
                self.omega = 0.0
                return

            # RSSI Gradient Tracking
            fwd_speed = 0.22
            spin_speed = 0.40

            if self.filtered_rssi >= self.previous_rssi:
                # Improving signal -> advance forward
                self.is_sweeping = False
                self.sweep_locked = False
                self.v = fwd_speed
                self.omega = 0.0
            else:
                # Worsening signal -> perform search sweep
                if not self.is_sweeping:
                    self.is_sweeping = True
                    self.sweep_start_time = now
                    self.best_sweep_rssi = self.filtered_rssi

                elapsed = now - self.sweep_start_time
                if self.filtered_rssi > self.best_sweep_rssi:
                    self.best_sweep_rssi = self.filtered_rssi

                if elapsed < self.sweep_duration:
                    self.v = 0.0
                    self.omega = spin_speed
                else:
                    self.is_sweeping = False
                    self.sweep_locked = True
                    self.previous_rssi = self.filtered_rssi
                    self.v = fwd_speed
                    self.omega = 0.0

        elif phase == AvoidPhase.BACK:
            elapsed = now - self.phase_start_time
            if elapsed < 0.8:
                self.v = -0.15
                self.omega = 0.0
            else:
                self.fsm_phase = AvoidPhase.ARC
                self.phase_start_time = now

        elif phase == AvoidPhase.ARC:
            elapsed = now - self.phase_start_time
            if elapsed < 1.5:
                self.v = 0.10
                self.omega = -0.55 if self.threat_side == "left" else 0.55
            else:
                self.fsm_phase = AvoidPhase.VERIFY
                self.phase_start_time = now

        elif phase == AvoidPhase.VERIFY:
            elapsed = now - self.phase_start_time
            if elapsed < 0.5:
                self.v = 0.08
                self.omega = 0.0
            else:
                # Check if still blocked
                if self.latest_range < self.obstacle_threshold:
                    self.threat_side = "right" if self.threat_side == "left" else "left"
                    self.consecutive_avoids += 1
                    if self.consecutive_avoids >= 3:
                        self.fsm_phase = AvoidPhase.RECOVERY
                        self.phase_start_time = now
                    else:
                        self.fsm_phase = AvoidPhase.BACK
                        self.phase_start_time = now
                else:
                    self.consecutive_avoids = 0
                    self.fsm_phase = AvoidPhase.RESUME
                    self.phase_start_time = now

        elif phase == AvoidPhase.RECOVERY:
            elapsed = now - self.phase_start_time
            if elapsed < 3.2:
                self.v = 0.0
                self.omega = 0.60
            else:
                self.consecutive_avoids = 0
                self.fsm_phase = AvoidPhase.RESUME

        elif phase == AvoidPhase.RESUME:
            self.previous_rssi = self.filtered_rssi
            self.is_sweeping = False
            self.fsm_phase = AvoidPhase.CLEAR

        elif phase == AvoidPhase.DELIVERED:
            self.v = 0.0
            self.omega = 0.0

    # -------------------------------------------------------------------------
    # Main Simulation Step (30 FPS)
    # -------------------------------------------------------------------------
    def _sim_tick(self):
        curr_time = time.time()
        dt = (curr_time - self.last_sim_time) * self.sim_speed
        self.last_sim_time = curr_time

        # Clamp dt to prevent tunneling on window drag
        dt = min(0.1, max(0.001, dt))

        if self.is_running:
            # 1. Update Dynamic Pedestrians
            for obs in self.obstacles:
                if obs.dynamic:
                    obs.x += obs.vx * dt
                    obs.y += obs.vy * dt
                    if obs.x < obs.x_bounds[0] or obs.x > obs.x_bounds[1]:
                        obs.vx *= -1.0
                    if obs.y < obs.y_bounds[0] or obs.y > obs.y_bounds[1]:
                        obs.vy *= -1.0

            # 2. Sensors
            self.previous_rssi = self.filtered_rssi
            self.raw_rssi, self.filtered_rssi = self._compute_rssi(dt)
            self.chart_raw_history.append(self.raw_rssi)
            self.chart_filt_history.append(self.filtered_rssi)

            self.latest_range = self._raycast_ultrasonic()

            # 3. Decision Engine (FSM)
            if not self.manual_mode:
                self._update_fsm(dt)

            # 4. Integrate Differential Drive Kinematics
            self.robot_theta += self.omega * dt
            # Normalize angle to -pi..pi
            self.robot_theta = (self.robot_theta + math.pi) % (2.0 * math.pi) - math.pi

            dx = self.v * math.cos(self.robot_theta) * dt
            dy = self.v * math.sin(self.robot_theta) * dt

            self.robot_x += dx
            self.robot_y += dy
            self.total_distance += math.hypot(dx, dy)

            # Bound inside café walls (-7.2 to 7.2, -5.7 to 5.7)
            self.robot_x = max(-7.2, min(7.2, self.robot_x))
            self.robot_y = max(-5.7, min(5.7, self.robot_y))

            # Record path trail
            self.trail.append((self.robot_x, self.robot_y))

        # 5. Render
        self._render_canvas()
        self._render_oscilloscope()
        self._render_telemetry()

        # Schedule next frame
        self.root.after(33, self._sim_tick)

    # -------------------------------------------------------------------------
    # Graphics Rendering
    # -------------------------------------------------------------------------
    def _render_canvas(self):
        self.canvas.delete("all")

        # 1. Wood Floor Texture (Grid Lines)
        for gx in range(-7, 8):
            c1 = self.w2c(gx, -6.0)
            c2 = self.w2c(gx, 6.0)
            self.canvas.create_line(c1[0], c1[1], c2[0], c2[1], fill="#34495e", width=1, dash=(2, 4))
        for gy in range(-5, 6):
            c1 = self.w2c(-7.5, gy)
            c2 = self.w2c(7.5, gy)
            self.canvas.create_line(c1[0], c1[1], c2[0], c2[1], fill="#34495e", width=1, dash=(2, 4))

        # 2. Perimeter Walls
        w_tl = self.w2c(-7.5, 6.0)
        w_br = self.w2c(7.5, -6.0)
        self.canvas.create_rectangle(w_tl[0], w_tl[1], w_br[0], w_br[1], outline="#bdc3c7", width=4)

        # Entrance Gap at South Wall (x = -0.75 to +0.75, y = -6.0)
        ent_l = self.w2c(-0.8, -6.0)
        ent_r = self.w2c(0.8, -6.0)
        self.canvas.create_line(ent_l[0], ent_l[1], ent_r[0], ent_r[1], fill="#2ecc71", width=5)
        self.canvas.create_text(self.w2c(0.0, -5.7)[0], self.w2c(0.0, -5.7)[1], text="ENTRANCE / CHARGING DOCK", fill="#2ecc71", font=("Segoe UI", 8, "bold"))

        # Kitchen Counter Area
        kc_tl = self.w2c(-2.0, 5.2)
        kc_br = self.w2c(2.0, 4.4)
        self.canvas.create_rectangle(kc_tl[0], kc_tl[1], kc_br[0], kc_br[1], fill="#d35400", outline="#e67e22", width=2)
        self.canvas.create_text(self.w2c(0.0, 4.8)[0], self.w2c(0.0, 4.8)[1], text="KITCHEN FOOD PREPARATION COUNTER", fill="white", font=("Segoe UI", 8, "bold"))

        # 3. Beacon Waves from Target Table
        bx, by = self.w2c(self.beacon.x, self.beacon.y)
        wave_phase = (time.time() * 3.0) % 1.0
        for r_factor in [15, 30, 45]:
            r_px = r_factor + (wave_phase * 15)
            alpha_color = "#2ecc71"
            self.canvas.create_oval(bx - r_px, by - r_px, bx + r_px, by + r_px, outline=alpha_color, width=1, dash=(3, 3))

        # 4. Obstacles (Tables, Chairs, Pedestrians)
        for obs in self.obstacles:
            cx, cy = self.w2c(obs.x, obs.y)
            # convert radius in m to pixels
            r_px = (obs.radius / self.world_w) * (self.canvas_w - 70)
            self.canvas.create_oval(cx - r_px, cy - r_px, cx + r_px, cy + r_px, fill=obs.color, outline="#ecf0f1", width=1)

            # Label for tables
            if "Table" in obs.name:
                self.canvas.create_text(cx, cy, text=obs.name, fill="white", font=("Segoe UI", 8, "bold"))

        # 5. Breadcrumb Trail
        if len(self.trail) > 1:
            for i in range(len(self.trail) - 1):
                p1 = self.w2c(self.trail[i][0], self.trail[i][1])
                p2 = self.w2c(self.trail[i+1][0], self.trail[i+1][1])
                self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill="#00d2d3", width=2)

        # 6. Ultrasonic Sensor Ray / Cone
        rx, ry = self.w2c(self.robot_x, self.robot_y)
        epx, epy = self.w2c(self.sensor_ray_endpoint[0], self.sensor_ray_endpoint[1])

        ray_color = "#e74c3c" if self.latest_range < self.obstacle_threshold else "#f1c40f"
        self.canvas.create_line(rx, ry, epx, epy, fill=ray_color, width=2, dash=(4, 2))
        self.canvas.create_oval(epx - 4, epy - 4, epx + 4, epy + 4, fill=ray_color, outline="white")

        # 7. Robot Chassis
        r_px = (self.robot_radius / self.world_w) * (self.canvas_w - 70)
        self.canvas.create_oval(rx - r_px, ry - r_px, rx + r_px, ry + r_px, fill="#e84118", outline="#f5f6fa", width=2)

        # Food Tray on Robot
        self.canvas.create_oval(rx - (r_px * 0.55), ry - (r_px * 0.55), rx + (r_px * 0.55), ry + (r_px * 0.55), fill="#f5cd79", outline="#303952")

        # Heading Pointer
        head_len = r_px + 10
        hx = rx + (head_len * math.cos(self.robot_theta))
        hy = ry - (head_len * math.sin(self.robot_theta))
        self.canvas.create_line(rx, ry, hx, hy, fill="white", width=3, arrow=tk.LAST)

    def _render_oscilloscope(self):
        self.chart_canvas.delete("all")
        cw = 460
        ch = 130

        # Background Grid (-80 dBm to -30 dBm)
        min_dbm = -80.0
        max_dbm = -28.0
        dbm_range = max_dbm - min_dbm

        # Draw grid lines at -70, -50, -36
        for dbm in [-70, -50, -36]:
            y_norm = 1.0 - ((dbm - min_dbm) / dbm_range)
            py = int(y_norm * ch)
            self.chart_canvas.create_line(0, py, cw, py, fill="#273c75", dash=(2, 4))
            self.chart_canvas.create_text(24, py - 6, text=f"{dbm} dBm", fill="#718093", font=("Consolas", 7))

        # Goal threshold line (-36 dBm)
        goal_y = int((1.0 - ((-36.0 - min_dbm) / dbm_range)) * ch)
        self.chart_canvas.create_line(0, goal_y, cw, goal_y, fill="#e74c3c", width=1, dash=(4, 2))

        # Draw Raw RSSI (Cyan)
        step = cw / float(len(self.chart_raw_history) - 1)
        raw_pts = []
        filt_pts = []

        for i, val in enumerate(self.chart_raw_history):
            px = i * step
            clamped = max(min_dbm, min(max_dbm, val))
            py = (1.0 - ((clamped - min_dbm) / dbm_range)) * ch
            raw_pts.extend([px, py])

        for i, val in enumerate(self.chart_filt_history):
            px = i * step
            clamped = max(min_dbm, min(max_dbm, val))
            py = (1.0 - ((clamped - min_dbm) / dbm_range)) * ch
            filt_pts.extend([px, py])

        if len(raw_pts) >= 4:
            self.chart_canvas.create_line(raw_pts, fill="#00d2d3", width=1)
        if len(filt_pts) >= 4:
            self.chart_canvas.create_line(filt_pts, fill="#4cd137", width=2)

    def _render_telemetry(self):
        # Update FSM Banner
        state_name, color = self.fsm_phase.value
        if self.fsm_phase == AvoidPhase.DELIVERED:
            banner_text = f"🎉 DELIVERED TO {self.beacon.name.upper()}! FOOD ARRIVED"
        elif self.manual_mode:
            banner_text = "🕹 MANUAL PILOT OVERRIDE MODE"
            color = "#8e44ad"
        else:
            banner_text = f"STATE: {state_name} — {'CLEAR PATH' if self.fsm_phase == AvoidPhase.CLEAR else 'OBSTACLE MANEUVER'}"

        self.fsm_card.configure(bg=color)
        self.fsm_state_label.configure(text=banner_text, bg=color)

        # Update Telemetry Labels
        self.lbl_pos.configure(text=f"{self.robot_x:+.2f} m, {self.robot_y:+.2f} m")
        deg = math.degrees(self.robot_theta) % 360.0
        self.lbl_heading.configure(text=f"{deg:.1f}°")
        self.lbl_vel.configure(text=f"{self.v:.2f} m/s, {self.omega:+.2f} rad/s")
        self.lbl_rssi.configure(text=f"{self.raw_rssi:.1f} dBm  /  {self.filtered_rssi:.1f} dBm [{self.filter_algorithm}]")

        dist_b = math.hypot(self.robot_x - self.beacon.x, self.robot_y - self.beacon.y)
        self.lbl_dist.configure(text=f"{dist_b:.2f} m ({self.beacon.name})")

        rng_text = f"{self.latest_range:.2f} m"
        if self.latest_range < self.obstacle_threshold:
            rng_text += " [ALERT: HAZARD]"
            self.lbl_range.configure(text=rng_text, fg="#e74c3c")
        else:
            rng_text += " [CLEAR]"
            self.lbl_range.configure(text=rng_text, fg="#4cd137")

        self.lbl_metrics.configure(
            text=f"{self.deliveries_count} Delivered | {self.avoidance_count} Hazards Cleared | Odom: {self.total_distance:.1f} m"
        )


# =============================================================================
# Main Entry Point
# =============================================================================
def main():
    root = tk.Tk()
    app = CafeSimulationApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
