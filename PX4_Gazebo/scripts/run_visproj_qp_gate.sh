#!/usr/bin/env bash
# IC2-5 SITL A/B gate for the Tier-1 QP + h-lead rewrite (commit e63751e2).
#   NEW arm = this tree (visibility_projection: Tier-1 convex QP with the
#             deliverability ball + penalised visibility slack folded in;
#             moving-target lead inert at CBF_DRIFT_TAU=0 default).
#   OLD arm = a git worktree at e63751e2^ (ccc41071): the prior baked
#             visibility_projection (alternating half-plane projection, post-hoc
#             lean cap) + its matching controller.py.
# Arms interleaved per rep so SITL temporal drift is shared. Cross-marker world.
# One SITL stack at a time (shared PX4/gz/MicroXRCEAgent) -> serial by construction.
# LANDING_OUT_BASE is set PER ARM so the two trees' autosaves never collide
# (the 2026-09-09 recovery lesson).
set -u
NEW_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"          # .../PX4_Gazebo (main tree)
OLD_ROOT="${OLD_ROOT:-$HOME/Soft-Precise-Landing-old/PX4_Gazebo}"
OLD_PARENT="${OLD_PARENT:-e63751e2^}"

TS="$(date +%Y%m%d-%H%M%S)"
BUNDLE="$NEW_ROOT/test_data/VisProjQPGate/$TS"
N_REPS="${N_REPS:-5}"
export HEADLESS=1
export WORLD=cross_marker MARKER_TYPE=cross
declare -A IC_ENU=( [IC2]="2.0,2.0,5.0" [IC3]="-2.0,2.0,5.0" [IC4]="2.0,2.0,7.0" [IC5]="2.0,2.0,3.0" )
PY="$HOME/ws/scripts/env2025/bin/python3"

if [ ! -d "$OLD_ROOT" ]; then
  echo "[qp_gate] OLD worktree missing: $OLD_ROOT"
  echo "  create it first:  git worktree add \"$(dirname "$(dirname "$OLD_ROOT")")/Soft-Precise-Landing-old\" $OLD_PARENT"
  exit 1
fi

mkdir -p "$BUNDLE"
SUMMARY="$BUNDLE/summary.tsv"
printf "ic\tarm\trep\tlanded\txy_err\trel_vel\tflight_s\tprecise\tsoft\ttarget_lost\tresult_dir\n" > "$SUMMARY"

score() {  # $1 = saved rep dir
  "$PY" - "$1" <<'PY'
import sys, os, numpy as np
d = sys.argv[1]
try:
    gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
    sp = gt.get("SoftPrecise", {})
    u = gt["UAV Pose"]; fs = len(u) / 60.0
    print(f"{sp.get('xy_err',0):.4f}\t{sp.get('rel_vel',0):.4f}\t{fs:.1f}\t"
          f"{int(sp.get('precise',False))}\t{int(sp.get('soft',False))}\t{int(sp.get('target_lost',False))}")
except Exception:
    print("-\t-\t-\t-\t-\t-")
PY
}

run_arm() {  # $1 ic  $2 arm(new|old)  $3 rep
  local ic="$1" arm="$2" rep="$3" root ld
  [ "$arm" = new ] && root="$NEW_ROOT" || root="$OLD_ROOT"
  ld="$BUNDLE/_autosave_${arm}"; mkdir -p "$ld"
  local before; before=$(ls -td "$ld/"*/ 2>/dev/null | head -1 || true)
  echo "=== $ic $arm rep=$rep  $(date +%H:%M:%S) ==="
  ( cd "$root" && env INITIAL_DRONE_ENU="${IC_ENU[$ic]}" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      LANDING_OUT_BASE="$ld" \
      bash "$root/scripts/run_aruco_landing_retry.sh" ) > "$BUNDLE/${ic}_${arm}_rep${rep}.log" 2>&1
  local latest; latest=$(ls -td "$ld/"*/ 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\t%s\t%s\tNO\t-\t-\t-\t-\t-\t-\t-\n" "$ic" "$arm" "$rep" >> "$SUMMARY"; return
  fi
  local dst="$BUNDLE/${ic}/${arm}/rep${rep}"; mkdir -p "$(dirname "$dst")"; cp -r "$latest" "$dst"
  printf "%s\t%s\t%s\tYES\t%s\t%s\n" "$ic" "$arm" "$rep" "$(score "$dst")" "$(basename "$dst")" >> "$SUMMARY"
}

echo "[qp_gate] NEW=$NEW_ROOT  OLD=$OLD_ROOT ($OLD_PARENT)  N_REPS=$N_REPS  bundle=$BUNDLE"
for ic in IC2 IC3 IC4 IC5; do
  for r in $(seq 1 "$N_REPS"); do
    run_arm "$ic" new "$r"; sleep 2
    run_arm "$ic" old "$r"; sleep 2
  done
done

echo; echo "[qp_gate] === summary ==="; column -t -s $'\t' "$SUMMARY"
"$PY" - "$SUMMARY" <<'PY'
import sys, csv, numpy as np
from collections import defaultdict
ag = defaultdict(lambda: dict(n=0, xy=[], ve=[], p=0, s=0, tl=0, nl=0))
for r in csv.DictReader(open(sys.argv[1]), delimiter='\t'):
    a = ag[(r['ic'], r['arm'])]
    if r['landed'] != 'YES': a['nl'] += 1; continue
    a['n'] += 1; a['p'] += int(r['precise']); a['s'] += int(r['soft']); a['tl'] += int(r['target_lost'])
    try: a['xy'].append(float(r['xy_err'])); a['ve'].append(float(r['rel_vel']))
    except ValueError: pass
print(f"\n{'cell':10} {'n':>2} {'mXY':>6} {'medXY':>6} {'maxXY':>6} {'maxVe':>6} {'P':>2} {'S':>2} {'TL':>2} {'noland':>6}")
pool = defaultdict(lambda: dict(xy=[], p=0, s=0, tl=0, nl=0))
for k in sorted(ag):
    a = ag[k]; xy = np.array(a['xy']) if a['xy'] else np.array([np.nan]); ve = np.array(a['ve']) if a['ve'] else np.array([np.nan])
    print(f"{k[0]+'/'+k[1]:10} {a['n']:>2} {np.nanmean(xy):>6.2f} {np.nanmedian(xy):>6.2f} {np.nanmax(xy):>6.2f} {np.nanmax(ve):>6.2f} {a['p']:>2} {a['s']:>2} {a['tl']:>2} {a['nl']:>6}")
    q = pool[k[1]]; q['xy'] += a['xy']; q['p'] += a['p']; q['s'] += a['s']; q['tl'] += a['tl']; q['nl'] += a['nl']
print()
for arm in ('new', 'old'):
    q = pool[arm]; xy = np.array(q['xy']) if q['xy'] else np.array([np.nan])
    print(f"POOL {arm}: n={len(q['xy'])} meanXY={np.nanmean(xy):.2f} medXY={np.nanmedian(xy):.2f} "
          f"maxXY={np.nanmax(xy):.2f} P={q['p']} S={q['s']} P+S={q['p']+q['s']} TL={q['tl']} noland={q['nl']}")
PY
echo "[qp_gate] DONE  ($(date +%H:%M:%S))  bundle: test_data/VisProjQPGate/$TS"
