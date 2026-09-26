#!/usr/bin/env bash
# Deploy the hardware controller fixes (docs/FIX_LOG.md FIX-004..FIX-015) from this repo to the Pi, in place, with backup + verification.
# Usage (Git Bash on Windows, from anywhere):   bash Hardware/deploy_to_pi.sh <pi-ip> [--dry-run]
# Only touches: controller.py flight_controller.py hardware_landing.py in /home/doctor/ws/scripts/precise_landing (backup: *.bak_before_fixdeploy_<ts>).
# Never run while a flight is in progress. Line endings: Pi controller.py/hardware_landing.py are LF, flight_controller.py is CRLF.
set -euo pipefail
IP="${1:?usage: deploy_to_pi.sh <pi-ip> [--dry-run]}"; DRY="${2:-}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; SRC="$REPO/Hardware/scripts"
P=/home/doctor/ws/scripts/precise_landing; TS=$(date +%Y%m%d_%H%M%S); TMP="$(mktemp -d)"
SSH="ssh -o ConnectTimeout=10 -o BatchMode=yes doctor@$IP"
FILES="controller.py flight_controller.py hardware_landing.py"

echo "== reachability"; $SSH "hostname && uptime -p" 2>&1 | grep -v -i "post-quantum\|warning\|upgrade\|vulnerable\|^\*\*"
echo "== a flight running on the Pi?"; $SSH "pgrep -af 'hardware_landing.py' || echo none" 2>/dev/null
echo "== current Pi hashes (before)"; $SSH "cd $P && md5sum $FILES" 2>/dev/null

for f in $FILES; do
  if [ "$f" = "flight_controller.py" ]; then sed 's/\r$//' "$SRC/$f" | sed 's/$/\r/' > "$TMP/$f"; else sed 's/\r$//' "$SRC/$f" > "$TMP/$f"; fi
  python3 -m py_compile "$TMP/$f" 2>/dev/null || python -m py_compile "$TMP/$f"
done
echo "== local (converted) hashes"; (cd "$TMP" && md5sum $FILES)
[ "$DRY" = "--dry-run" ] && { echo "dry run: nothing written"; exit 0; }

for f in $FILES; do
  $SSH "cd $P && cp $f $f.bak_before_fixdeploy_$TS && cat > $f.new" < "$TMP/$f" 2>/dev/null
  $SSH "cd $P && /home/doctor/denv/bin/python3 -m py_compile $f.new && mv $f.new $f" 2>/dev/null
done
echo "== Pi hashes (after) - must equal the local hashes above"; $SSH "cd $P && md5sum $FILES" 2>/dev/null
echo "== import check on the Pi + key markers"
$SSH "cd $P && /home/doctor/denv/bin/python3 -c 'import controller, flight_controller; print(\"imports OK\")' && grep -c '_levelled_basis' controller.py && grep -c 'PLASMC_THRUST_TILT_COMP' controller.py && grep -c '_imuTouchdownStep' flight_controller.py && grep -c 'HW_MIN_FLIGHT_V' hardware_landing.py && grep -c 'FC_IMU_TD_ARM_FALLBACK_S' hardware_landing.py" 2>&1 | grep -v -i "post-quantum\|warning\|upgrade\|vulnerable\|^\*\*\|Flight Controller is disc"
echo "backups: *.bak_before_fixdeploy_$TS in $P"
