#!/usr/bin/env bash
# Launcher for the impulse-response rate-loop test.
# Reuses run_aruco_landing.sh's infrastructure (SITL + bridges + QGC),
# but swaps the Python script via the PY_SCRIPT env hook added 2026-05-22.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Impulse test takes ~25 s of flight (3s stabilize + 9 impulses × 1.3s + 3s land).
# Wall-clock budget: 90s arm/EKF settle + 25s flight + cleanup ≈ 130s.
export PY_TIMEOUT_S="${PY_TIMEOUT_S:-150}"
export PY_SCRIPT="impulse_response.py"
# Use the retry wrapper to absorb PX4 SITL lockstep failures (is_armable
# never going True is a known race; cheap retry is the standard fix).
exec bash "$SCRIPT_DIR/run_aruco_landing_retry.sh"
