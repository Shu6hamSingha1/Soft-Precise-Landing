#!/usr/bin/env bash
# Terminal-kick approach config (docs/TERMINAL_KICK_COMMIT_DESIGN.md) — thin preset over
# run_aruco_landing.sh. Bundles Part 1 (restore lateral convergence authority in h_d) + Part 2
# (corner-exit terminal commit) into one launch. Exports the integrated env, then delegates.
# Every value is overridable (`env PLASMC_HD_KR=0.5 bash scripts/run_terminal_approach.sh`) so the
# deferred sweeps just re-run this with the swept var set. Pass-through args/env reach the launcher
# (e.g. HEADLESS=1, N_REPS=...).
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Validated frontier baseline (feedback_kappa0_unfreezes_lateral, 2026-06-26 — NOT baked) ─────
# Run Part 1 + commit ON the validated lateral-convergence frontier, NOT the older baked VDF
# auto-align (kappa_0=0.125 frozen-κ, χ_r=0.5 under-damped, XI2=0.2/P20=25 loose). ⚠ setting any
# PLASMC_XI2_*/KAPPA0_* env bypasses that group's auto-align — so all three axes are set explicitly.
export PLASMC_KAPPA0_X="${PLASMC_KAPPA0_X:-0.5}"   # un-freeze lateral κ (vs baked 0.125)
export PLASMC_KAPPA0_Y="${PLASMC_KAPPA0_Y:-0.5}"
export PLASMC_KAPPA0_Z="${PLASMC_KAPPA0_Z:-0.25}"
export PLASMC_CHI_R_X="${PLASMC_CHI_R_X:-1.5}"     # velocity-damping D-term (kills the hump; vs baked 0.5)
export PLASMC_CHI_R_Y="${PLASMC_CHI_R_Y:-1.5}"
export PLASMC_XI2_X="${PLASMC_XI2_X:-0.7}"         # flow-funnel contraction rate (vs baked 0.2)
export PLASMC_XI2_Y="${PLASMC_XI2_Y:-0.7}"
export PLASMC_XI2_Z="${PLASMC_XI2_Z:-1.0}"
export PLASMC_P20_X="${PLASMC_P20_X:-15}"          # flow-funnel initial width (vs baked 25)
export PLASMC_P20_Y="${PLASMC_P20_Y:-15}"
export PLASMC_P20_Z="${PLASMC_P20_Z:-10}"

# ── Part 1 — restore convergence authority in h_d (un-degenerate ζ_h) ───────────────────────────
# h_d = measured ṡ copies the flow (h_e≈0, never commands "stop") → residual v_res → terminal kick.
export PLASMC_HD_FUNNEL_REF="${PLASMC_HD_FUNNEL_REF:-1}"  # h_d uses the funnel reference, not measured ṡ
export PLASMC_HD_KR="${PLASMC_HD_KR:-0.3}"               # inward convergence term (restores p_r·Ṡ_r the
                                                          # pure funnel-ref drops). SWEEP {0.3,0.5,0.7,1.0}
                                                          # NB: HD_KR only acts with HD_FUNNEL_REF=1.
export PLASMC_P2INF_X="${PLASMC_P2INF_X:-0.12}"          # tight flow-funnel floor → bounds h_e_xy (=v_res
export PLASMC_P2INF_Y="${PLASMC_P2INF_Y:-0.12}"          # post-switch). SWEEP down to the chatter floor.
# Optional: lower χ_r so k_r carries convergence and χ_r carries damping (avoid double-drive over-
# drive). Left at the baked default; uncomment / override during the χ_r–k_r balance sweep.
# export PLASMC_CHI_R_X="${PLASMC_CHI_R_X:-0.5}"; export PLASMC_CHI_R_Y="${PLASMC_CHI_R_Y:-0.5}"

# ── Part 2 — corner-exit terminal commit (zero ζ_r, smooth taper) ───────────────────────────────
export PLASMC_TERMINAL_COMMIT="${PLASMC_TERMINAL_COMMIT:-1}"
export PLASMC_TC_EXTENT="${PLASMC_TC_EXTENT:-400}"      # corner-exit: marker px extent (≈ Z 0.5 m)
export PLASMC_TC_SEN="${PLASMC_TC_SEN:-0.3}"            # |s_e_n| centered cut
export PLASMC_TC_DSEN="${PLASMC_TC_DSEN:-0.2}"          # |ds_e_n| settled cut (the v_res guard)
export PLASMC_TC_FRAMES="${PLASMC_TC_FRAMES:-3}"        # consecutive-frame confirm
export PLASMC_TC_RAMP_S="${PLASMC_TC_RAMP_S:-0.3}"     # smooth ζ_r→0 taper (no a_u jolt)

# ── the old app-side proximity commit is replaced by the controller commit ──────────────────────
export LANDING_COMMIT_EXTENT="${LANDING_COMMIT_EXTENT:-0}"

# ── first tests are GT-based (overridable → 0 for the perception run) ───────────────────────────
export PLASMC_GT_FEEDBACK="${PLASMC_GT_FEEDBACK:-1}"

echo "[run_terminal_approach] Frontier: KAPPA0_xy=$PLASMC_KAPPA0_X CHI_R=$PLASMC_CHI_R_X" \
     "XI2_xy=$PLASMC_XI2_X P20_xy=$PLASMC_P20_X | Part1: HD_FUNNEL_REF=$PLASMC_HD_FUNNEL_REF" \
     "HD_KR=$PLASMC_HD_KR P2INF_xy=$PLASMC_P2INF_X | Part2: TERMINAL_COMMIT=$PLASMC_TERMINAL_COMMIT" \
     "TC_SEN=$PLASMC_TC_SEN TC_DSEN=$PLASMC_TC_DSEN TC_RAMP_S=$PLASMC_TC_RAMP_S |" \
     "GT_FEEDBACK=$PLASMC_GT_FEEDBACK"
exec bash "$SCRIPT_DIR/run_aruco_landing.sh" "$@"
