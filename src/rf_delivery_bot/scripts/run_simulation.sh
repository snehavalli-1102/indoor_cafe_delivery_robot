#!/usr/bin/env bash
# run_simulation.sh — Launch the RF Delivery Bot simulation and record a ROS2 bag.
#
# Usage:
#   chmod +x scripts/run_simulation.sh   # make executable (run once)
#   ./scripts/run_simulation.sh
#
# Press Ctrl+C to stop all processes cleanly.

set -euo pipefail

# ── 1. Source ROS2 Humble ─────────────────────────────────────────────────────
# shellcheck disable=SC1091
. /opt/ros/humble/setup.bash

# ── 2. Source the workspace overlay ──────────────────────────────────────────
# shellcheck disable=SC1091
. ~/ros2_ws/install/setup.bash

# ── 3. Create logs directory ──────────────────────────────────────────────────
mkdir -p ~/ros2_ws/logs

# ── 4. Launch the full system in the background ───────────────────────────────
echo "[run_simulation] Starting RF Delivery Bot launch system …"
ros2 launch rf_delivery_bot delivery_launch.py &
LAUNCH_PID=$!
echo "[run_simulation] Launch PID: ${LAUNCH_PID}"

# ── 5. Wait for Gazebo and nodes to initialise ────────────────────────────────
echo "[run_simulation] Waiting 5 s for system initialisation …"
sleep 5

# ── 6. Start ros2 bag recording in the background ────────────────────────────
BAG_NAME="delivery_run_$(date +%Y%m%d_%H%M%S)"
echo "[run_simulation] Starting bag record → ~/ros2_ws/logs/${BAG_NAME}"
ros2 bag record \
    -o ~/ros2_ws/logs/"${BAG_NAME}" \
    /filtered_rssi \
    /cmd_vel \
    /ultrasonic/range &
BAG_PID=$!
echo "[run_simulation] Bag PID: ${BAG_PID}"

# ── 7. Status message ─────────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════════════"
echo "  System running. Press Ctrl+C to stop."
echo "  Bag output: ~/ros2_ws/logs/${BAG_NAME}"
echo "════════════════════════════════════════════════════════"
echo ""

# ── 8. Trap SIGINT — clean shutdown ──────────────────────────────────────────
cleanup() {
    echo ""
    echo "[run_simulation] Caught Ctrl+C — shutting down …"

    if kill -0 "${BAG_PID}" 2>/dev/null; then
        echo "[run_simulation] Stopping bag recorder (PID ${BAG_PID}) …"
        kill "${BAG_PID}" 2>/dev/null || true
    fi

    if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo "[run_simulation] Stopping launch process (PID ${LAUNCH_PID}) …"
        kill "${LAUNCH_PID}" 2>/dev/null || true
    fi

    # Give processes a moment to exit gracefully before force-kill
    sleep 1
    kill -0 "${BAG_PID}"    2>/dev/null && kill -9 "${BAG_PID}"    2>/dev/null || true
    kill -0 "${LAUNCH_PID}" 2>/dev/null && kill -9 "${LAUNCH_PID}" 2>/dev/null || true

    echo "[run_simulation] Shutdown complete."
    exit 0
}

trap cleanup SIGINT SIGTERM

# ── 9. Wait for both background processes ─────────────────────────────────────
wait "${LAUNCH_PID}" "${BAG_PID}"
