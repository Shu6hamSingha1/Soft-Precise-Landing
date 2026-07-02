---
name: project_terminal_kick_commit_design
description: "⭐⭐ FINALIZED terminal-kick design (2026-06-28, user-led, GT-FB) — full spec in PX4_Gazebo/docs/TERMINAL_KICK_COMMIT_DESIGN.md. Root: residual lateral VELOCITY v_res (commanded, NOT passive — h_d=measured s_dot copies the flow, corr 0.90, so h_e≈0, controller never says 'stop'), 1/Z-normalized -> s_e_n breach -> zeta_r->inf -> kick. Two-part fix: (1) APPROACH restore convergence via HD_FUNNEL_REF=1 + HD_KR=k_r (pure funnel-ref is INERT — prescribes S_r const; k_r restores the dropped p_r*Sdot_r term) + tight P2INF_xy~0.12 + maybe lower chi_r; (2) COMMIT at corner-exit (MARKER_EXTENT>=400px ~= Z0.5m, start-height-INDEPENDENT) = zero zeta_s + ring-flow handoff + one-way latch. Discriminator: |s_e_n| small AND h_z<0 AND |ds_e_n| small (settled), held N frames -> case(b) commit; off-center/ds_e_n>0 growing -> case(a) abort+re-ascend. Dropped ||h_xy|| guard for |ds_e_n| (same v_res info, centroid-robust, catches zero-crossing). h_rd FIXED."
metadata:
  node_type: memory
  type: project
  originSessionId: 13b15a0c-9903-4e62-95a5-6161b516a73e
---

## Finalized terminal-kick approach (2026-06-28, user-led, GT-FB). Full doc: `PX4_Gazebo/docs/TERMINAL_KICK_COMMIT_DESIGN.md`.

**ROOT (settled, data-backed on 620 Landing_Test reps).** The kick = residual lateral VELOCITY
`v_res`, 1/Z-normalized: `s_e_n=lat/(Z·p_10)` → breach → `ζ_r→∞` → `a_u_xy~1000` → max tilt →
fling+thrust-steal → fly. `v_res` is **commanded, not passive**: `h_d=measured ṡ` copies the flow
(`corr(||h_d||,||h||)=0.90`), so `h_e≈0`, no restoring command — even dead-centered `h_d≈0.11`
("never says stop"). Equilibrium: `lat*=v_res/(k_lat−|h_rd|)≠0` → `s_e_n*→∞` as `Z→0`, so faster
gain can't save it. **Position does NOT discriminate** (precise |s_e_n| 0.163 vs failed 0.159);
**velocity does** (v_res 0.067 vs 0.234; |ds_e_n| 0.061 vs 0.252; ||h_xy|| 0.132 vs 0.390).

**PART 1 — APPROACH: restore convergence authority in h_d.**
- `s_dot_meas` (current default) = degenerate copy → no drive.
- pure funnel-ref `p_10·S_r·dp_r` = **INERT** — it prescribes `ζ̇_r=0` (holds S_r constant; drops
  the convergence term `p_r·Ṡ_r` of the product rule `ṙ=p_r·Ṡ_r+S_r·ṗ_r`); position funnel is
  near-static (p_r 1.2→1.0, ξ_r=0.10, dp_r≈0.01) → `_hd_rate≈0.0016` (measured, 151× < s_dot copy).
- **`HD_FUNNEL_REF=1` + `HD_KR=k_r`** = restores `p_r·Ṡ_r` via `−k_r·ζ_r/g_r` ⇒ inward command
  `≈−p_10·k_r·p_r·(1−S_r²)·artanh(S_r)`: drives S_r→0 AND →0 at center. Acts ∝ offset (negligible
  centered, strong off-center: IC4 s_e_n@0.5 0.76→0.07 at k_r=0.3).
  - ⚠ **CODE COUPLING:** HD_KR only acts inside `if hd_funnel_ref` (controller.py ~1061);
    `HD_KR=1` with `FUNNEL_REF=0` is SILENTLY IGNORED → must set BOTH.
  - ⚠ k_r term rides into `dh_d` (full h_d differentiated under funnel-ref, ~1140); edge-derivative
    spike (DH_D_MAX clamps).
- `P2INF_xy → ~0.12` (sweep to lag floor) bounds `h_e_xy`. Maybe lower `χ_r` (k_r + χ_r·ζ_r are two
  convergence routes → over-drive risk; let k_r converge, χ_r damp).

**PART 2 — COMMIT at corner-exit (terminal).**
- Event = `MARKER_EXTENT_PX≥~400` (corners at 640×480 border) ≈ Z 0.50m, IQR ±0.05 — **fixed
  altitude, start-height-INDEPENDENT** (fires when marker angular extent = FoV). Replaces the
  start-dependent loom-integral. Gate on corner PIXEL-POSITION-near-border, NOT count-dropped
  (mid-frame vanish = decode fail → KLT-bridge; border vanish = geometric exit).
- **Discriminator (FINALIZED):** `|s_e_n| small AND h_z<0 (genuine loom) AND |ds_e_n| small
  (settled)`, held N frames → **case (b) COMMIT** (one-way latch). `|s_e_n| large OR ds_e_n>0
  growing` → **case (a) ABORT → ascend → re-attempt** (new behavior; fly→retry).
- **Dropped `||h_xy||<0.15` guard for `|ds_e_n|`:** `|ds_e_n|≈v_res/(Z·p_10)` when centered = same
  velocity info, but centroid-derived (decode-robust > LK-flow near deck), same s_e_n coordinate,
  and catches the ZERO-CROSSING miss (|s_e_n| small but whipping through center). Sign maps to the
  split: rate race ⇒ safe iff ds_e_n≤0; ds_e_n>0 growing = breach precursor. Validated ≈4× sep.
  ⚠ ds_e_n is 1/Z-amplified + noisy near deck → MUST filter (smooth4/KF) + N-frame hold; threshold
  not a transferable constant.
- **Case (b) terminal:** zero `ζ_s/s_e_n` (removes the ζ_r blow-up; honest — marker invisible <0.3m),
  keep vertical(loom)+yaw via RINGS, lateral held by flow funnel IF not breached (verify in test).

**VELOCITY BOUND — exact (corrected).** `p_2inf` bounds `h_e_xy=h−h_d`, NOT raw `||h_xy||`. Holds at
center because `h_d→0` there (k_r→0, ff→0) ⇒ `h_e_xy≈h_xy≈v_res`. ⚠ **`h_e_xy` is NOT "translational
velocity with rotation compensated out"** (I claimed this, FALSIFIED): `rot=cross(w,s)` FF is only
~19% of h (0.041 vs 0.222), too small to cancel rotation; h corr w_xy 0.75 > v_lat/Z 0.66 (partly
tilt-accel confound). h_e_xy ≈ raw flow at center; bound works by `h_d→0`, NOT rotation removal.
Floor: P2INF_xy can't go below lag-limited achievable h_e (else funnel breach→chatter); 0.12 = sweep
target. Metric flips: pre-switch use raw ||h_xy|| (h_e degenerate≈0); post-switch read h_e_xy + |ds_e_n|.

**INVARIANTS:** h_rd=−0.42 fixed; scale/depth-free (all triggers image-space; Z analysis-only). This
LIVE-to-touchdown design = the moving/rover path; should also beat open-loop loom-commit for stationary.

**IMPLEMENTED (2026-06-28, controller, env-gated `PLASMC_TERMINAL_COMMIT=1` default-off).**
`_terminalCommitStep()` called from TOP of `PLASMC()` (after `_updateOptFlow` → h_z FRESH, no stale
lag; sets `_committed` before the surface assembly same step). Inputs s_e_n=`_s_e_n[-1]`,
ds_e_n=`_s_dot_meas[-1]/p_10`. case(b) → fade `ζ_r,dζ_r→0` in σ_xy over `PLASMC_TC_RAMP_S` (0.3s
TAPER, not hard-zero — hard zero steps σ_xy by χ_r·ζ_r~0.27 → a_u jolt). case(a) → `_abort_requested`
(+`ABORT_REQUESTED` property; re-ascend app module DEFERRED). Knobs: TC_EXTENT 400 / TC_SEN 0.3 /
TC_DSEN 0.2 / TC_FRAMES 3 / TC_RAMP_S 0.3 (reuses COMMIT_WIN 7). **`LANDING_COMMIT_LOOM` REMOVED
ENTIRELY from landing_test.py (user: wrong approach)** — supersedes [[feedback_loom_commit_gate]].
First test: `PLASMC_TERMINAL_COMMIT=1 LANDING_COMMIT_EXTENT=0`, watch for "case(b) COMMIT" print.

**⚠ HANDOFF — GT-FB vs perception (user-flagged).** In GT-FB (`PLASMC_GT_FEEDBACK=1`) the GT flow is
CONTINUOUS to touchdown (gt_feedback replaces s/h; perception still runs so MARKER_EXTENT_PX corners
fire the trigger). So GT-FB has **NO corner→ring flow handoff to test** — only the CONTROL handoff
(σ_xy drops ζ_r). GT-FB validates: (1) handoff SMOOTHNESS (no a_u/σ step at commit — the TC_RAMP_S
taper). GT-FB does NOT validate: (2) ring-flow RELIABILITY below ~0.5m (GT flow never degrades =
false-confidence trap) → SEPARATE perception-ON validation (ring tracks GT-flow at close range +
σ_xy=ζ_h doesn't breach flow funnel with corners gone). GT-FB success = necessary NOT sufficient.

**⭐ SITL TEST FINDINGS (2026-06-28, GT-FB, user-led — n=1 each, high variance).**
- **Commit fires correctly + depth-free CONFIRMED.** case(b) latches on image-space only
  (MARKER_EXTENT_PX, |s_e_n|, h_z, |ds_e_n|) — NO altitude/Z in the commit path. Part 1
  (HD_FUNNEL_REF=1 + HD_KR=0.3 + P2INF_xy=0.12) drives v_res to ~0.02 m/s at the 0.5 m commit
  (excellent convergence; lat 0.019).
- **⛔ a_u→0 TAPER = DEAD-END, REMOVED.** I added a post-commit lateral-a_u taper as a band-aid
  (test1 showed zeroing ζ_r alone relocated the kick). WRONG: (a) it's the open-loop level descent
  = the rejected loom-commit, stationary-only; (b) it MASKED the real driver so the frontier re-run
  couldn't test it. Removed entirely from controller.py. Don't re-add.
- **⭐ THE REAL TERMINAL a_u DRIVER = the c-term loom×flow** `−(h·ê3)·h = −h_z·h_xy ~ (vz·vlat)/Z²`
  — the STRONGEST blow-up term. Decomposed a_u = reach(Γσ) + switch(θκ) + c-term: with CORRECT
  params (Γ=0.4375 auto-aligned, NOT the bare 2.0) the reaching stays SMALL (<0.8); the c-term
  dominates (9.5 of 15.9 at 0.10 m) AND inflates θ (1.8→12.3) which then drives switching. The
  c-term is computed from RAW FLOW h, INDEPENDENT of ζ_r/ζ_h → that's why zeroing a barrier
  relocates the blow-up (the barriers were never the driver). NOT a fundamental 1/Z wall — a
  specific, addressable term (cterm_loom_scale / cap / terminal gate). Fix at the SOURCE.
- **CONFIG CONFOUND LESSON.** First runs used the OLD baked VDF auto-align (kappa_0=0.125 frozen-κ,
  χ_r=0.5, XI2=0.2, P20=25), NOT the validated frontier (kappa_0=0.5/χ_r=1.5/XI2=0.7/P20=15,
  [[feedback_kappa0_unfreezes_lateral]] — never baked). run_terminal_approach.sh now wires the
  frontier gains explicitly (⚠ setting any XI2/KAPPA0 env bypasses that group's auto-align → set
  all 3 axes). Always verify logged Control_Params vs the intended baseline before diagnosing.
- **CLEANUP (user-led):** LANDING_COMMIT_LOOM REMOVED from app; 42 loom-commit ICValidation gates
  DELETED (1.1 GB, incl the k_r=0.3 042607 gate) as invalid; a_u-taper session runs deleted.
  [[feedback_loom_commit_gate]] stamped SUPERSEDED/INVALID (kept for history).
- **OPEN:** with the taper gone, does the c-term still detonate on the frontier baseline (test4,
  un-analyzed)? Next = n=3-5 frontier batch + attack the c-term at source. Vertical creep
  (exponential h_rd never touches down → long coast) + touchdown-disarm are SEPARATE downstream
  issues surfaced by test3.

**DEFERRED (build/tune, not design):** k_r sweep {0.3,0.5,0.7,1.0} on IC4 @ Z0.5 (ignore sub-0.5,
commit cuts it; guard no pre-0.5 overshoot); P2INF_xy floor sweep; χ_r/k_r balance; commit
thresholds; case(a) abort module; ring-handoff validation; first test = commit on
|s_e_n|+h_z<0+|ds_e_n| then read post-commit touchdown. Builds on
[[feedback_terminal_kick_commit_vs_live]], [[feedback_kappa0_unfreezes_lateral]],
[[feedback_loom_commit_gate]], [[feedback_flow_funnel_zetah_works]].
