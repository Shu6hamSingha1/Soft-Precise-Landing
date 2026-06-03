#!/usr/bin/env bash
# Phase 1 launcher — Windows / Git Bash side.
#
# Workflow (the script does steps 1, 3, 4; the user does step 2):
#   1. Pull latest from origin/main
#   2. *** USER ACTION *** In MATLAB:
#         cd MATLAB/Multi_init_cond
#         phase1_baseline_sweep
#      (~3-5 min; saves Datasets/Phase1/phase1_summary.mat + done.flag)
#   3. This script polls for done.flag and auto-commits when it appears
#   4. Pushes the summary.mat to origin/main
#
# Usage (from repo root in Git Bash):
#     bash MATLAB/Multi_init_cond/run_phase1_windows.sh
#
# If MATLAB / gh aren't found, add to PATH first:
#     export PATH=$PATH:"/c/Program Files/GitHub CLI"

set -u

# Find the repo root (script may be run from anywhere)
REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null)"
if [ -z "$REPO_ROOT" ]; then
  echo "ERROR: not inside a git repo. cd to Soft-Precise-Landing/ first."
  exit 1
fi
cd "$REPO_ROOT"

PHASE1_DIR="MATLAB/Datasets/Phase1"
DONE_FLAG="$PHASE1_DIR/done.flag"
SUMMARY="$PHASE1_DIR/phase1_summary.mat"
POLL_INTERVAL="${POLL_INTERVAL:-10}"          # seconds between checks
MAX_WAIT_S="${MAX_WAIT_S:-1800}"               # 30 min cap

# Step 1: pull
echo "[phase1] git pull origin main ..."
git pull --ff-only origin main || { echo "git pull failed"; exit 1; }

# Wipe any stale sentinel so we don't pick up an old run
rm -f "$DONE_FLAG"

cat <<EOF

================================================================================
PHASE 1 — WAITING FOR MATLAB
================================================================================

  In MATLAB now run:

      cd MATLAB/Multi_init_cond
      phase1_baseline_sweep

  (16 simulations, ~3-5 min.  Output: $SUMMARY)

  Polling every ${POLL_INTERVAL}s for $DONE_FLAG ...
  (Ctrl-C to abort)

EOF

# Step 3: poll for done.flag
elapsed=0
while [ ! -f "$DONE_FLAG" ]; do
  sleep "$POLL_INTERVAL"
  elapsed=$((elapsed + POLL_INTERVAL))
  printf "  ... waited %ds\r" "$elapsed"
  if [ "$elapsed" -ge "$MAX_WAIT_S" ]; then
    echo
    echo "[phase1] TIMEOUT after ${MAX_WAIT_S}s — no done.flag.  MATLAB may have crashed."
    echo "         Check MATLAB Command Window for errors."
    exit 1
  fi
done
echo
echo "[phase1] done.flag detected (`cat "$DONE_FLAG"`)"

# Sanity check the summary file exists
if [ ! -f "$SUMMARY" ]; then
  echo "[phase1] ERROR: done.flag exists but $SUMMARY does not. Inspect MATLAB output."
  exit 1
fi

# Print local size for sanity
sz=$(stat -c%s "$SUMMARY" 2>/dev/null || stat -f%z "$SUMMARY" 2>/dev/null)
echo "[phase1] $SUMMARY ($sz bytes)"

# Step 4: commit & push
echo "[phase1] git add + commit + push ..."
git add "$SUMMARY"

# Use the done.flag timestamp in the commit message for traceability
ts=$(cat "$DONE_FLAG")
git commit -m "Phase 1 results: MATLAB baseline summary ($ts)" || {
  echo "[phase1] git commit failed (maybe no changes? maybe summary unchanged?)"
  exit 1
}

git push origin main || { echo "[phase1] git push failed"; exit 1; }

# Clean up sentinel locally (it's gitignored anyway)
rm -f "$DONE_FLAG"

echo
echo "================================================================================"
echo "  PHASE 1 RESULTS PUSHED."
echo "  Ping the Linux side — the analyzer will pick this up automatically."
echo "================================================================================"
