#!/usr/bin/env bash
# Linux side of Phase 1 automation.  Polls origin/main for the MATLAB
# summary.mat that the Windows wrapper pushes.  When detected, pulls
# and runs the analyzer.
#
# Usage:
#     bash PX4_Gazebo/poll_phase1.sh
#
# Env:
#     POLL_INTERVAL  seconds between polls (default 30)
#     MAX_WAIT_S     overall budget (default 3600 = 1 hour)

set -u

REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null)"
if [ -z "$REPO_ROOT" ]; then
  echo "ERROR: not inside a git repo"; exit 1
fi
cd "$REPO_ROOT"

TARGET="MATLAB/Datasets/Phase1/phase1_summary.mat"
POLL_INTERVAL="${POLL_INTERVAL:-30}"
MAX_WAIT_S="${MAX_WAIT_S:-3600}"
VENV_PY="$HOME/ws/scripts/env2025/bin/python3"

echo "[poll] watching origin/main for $TARGET (every ${POLL_INTERVAL}s, up to ${MAX_WAIT_S}s)"

elapsed=0
while [ "$elapsed" -lt "$MAX_WAIT_S" ]; do
  # `git fetch` is cheap and doesn't touch the working tree
  git fetch origin main --quiet 2>/dev/null || true
  if git cat-file -e "origin/main:$TARGET" 2>/dev/null; then
    echo "[poll] $TARGET found on origin/main"
    break
  fi
  sleep "$POLL_INTERVAL"
  elapsed=$((elapsed + POLL_INTERVAL))
  printf "[poll] waited %ds\r" "$elapsed"
done

if ! git cat-file -e "origin/main:$TARGET" 2>/dev/null; then
  echo
  echo "[poll] timeout after ${MAX_WAIT_S}s — $TARGET still missing on origin/main"
  exit 1
fi

echo
echo "[poll] pulling and analyzing..."
git pull --ff-only origin main
echo
echo "================================================================================"
echo "  PHASE 1 ANALYZER OUTPUT"
echo "================================================================================"
"$VENV_PY" PX4_Gazebo/analyze_matlab_phase1.py MATLAB/Datasets/Phase1/
