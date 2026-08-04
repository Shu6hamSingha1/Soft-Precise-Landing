# **************************************************************************
# PLASMC: Performance-constrained Leakage-type Adaptive Sliding-Mode Control
#
# Manuscript-faithful PX4 port. Aligned to MATLAB single-run reference:
#   MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m
# and the analytical formulation in Soft_Precise_Landing/control_formulation.tex
# (Sections III-A outer/middle loop, III-B1 virtual-compass yaw ASMC,
# III-B2 geometric SO(3) tracker).
#
# Pipeline (per tick at ~50 Hz):
#   1. Outer Virtual Image Point PID:   pixel error  -> V_ds_d              §III-A1
#   2. Optic-Flow Funnel + ASMC:        V_h_e        -> u_h (= a_u)         §III-A2
#   3. Yaw ASMC (virtual compass):      alpha_e      -> psi_d_dot -> psi_d  §III-B1
#   4. R_d construction:               -I_a + a_h(psi_d) -> Gram-Schmidt    §III-B2
#   5. SO(3) attitude error:            e_R = ½ vee(R_d^T R - R^T R_d)      §III-B2
#   6. Body-rate setpoint:              w_u = -K_R · e_R    (rate-mode P)
#   7. Thrust scalar:                   B_T = m · |I_a[2] + g|
#
# Rate-mode caveat: we keep PX4's body-rate + thrust interface (MAVSDK
# set_attitude_rate), so the inner SO(3) law is reduced to its proportional
# part `w_u = -K_R · e_R`. The damping `-k_Omega · e_Omega` and gyroscopic
# feedforward `omega × J · omega` from the manuscript torque law are handled
# inside PX4's onboard rate controller (the body-rate setpoint we ship is
# tracked there).
#
# Legacy Euler-PD + direct-yaw path (the older PX4 implementation) is
# archived at Obsolete/src/controller_v0.py (and in git history).
#
# Notes on Gazebo vs MATLAB:
#   - Camera intrinsics: 640x480 @ fx=fy=270 (Gazebo SDF, hfov=1.74 rad).
#     MATLAB uses 320x240 @ f=135. Same hfov → image-normalized PLASMC gains
#     are invariant; only pixel-space quantities (rho_fov) scale 2x.
#   - Image cals live in img_data.py (single source of truth): _sensor_cal_hw
#     (corner flow), _sensor_cal_ring (ring flow), _sensor_cal_s (centroid).
#     The controller consumes ONE already-calibrated flow via getOptFlowAngVel():
#     the corner KF by default, or — with FLOW_FUSE_RING=1 — the corner+ring
#     fusion EKF's target-relative [h_tr; w]. The fused signal is a SINGLE
#     calibrated quantity in the corner-cal (_sensor_cal_hw) units (the ring is
#     transfer-matched to that scale, and the two source cals + EKF are internal
#     to img_data); the controller applies no cal of its own. Current cal of
#     record: all-13-run phased corner M + re-keyed transfer ring M_ring
#     (2026-06-07; supersedes the 8-run 2026-06-06 cal). FLOW_FUSE_RING default
#     ON (EKF fused flow) as of 2026-06-07.
# **************************************************************************
import os
import time
import numpy as np
from scipy.linalg import expm
from scipy.signal import savgol_filter
from threading import Thread
from collections import deque

from numerical_methods import RK5, smooth4
import img_data as ID  # img_data_hw.py no longer exists; img_data.py is the hardware IMG_PROCESSOR
from ahrs import Quaternion
from cbf_visibility import cbf2_filter

SLEEP_TIME = 1/200
N_DIM = 3
e3 = np.eye(N_DIM)[:, 2]

mass = 2.114  # kg, Holybro X500 (matches MATLAB Constants.m: m=2.114)
g = 9.80      # m/s^2 (matches Gazebo aruco.sdf <gravity>0 0 -9.8</gravity>;
              # MATLAB Constants.m uses 9.81 internally — sub-0.1% difference,
              # the 9.80 value here is correct for SITL physics)

# Clamp |S| < 1 - S_MARGIN to keep log-barrier finite (MATLAB uses 0.05 margin)
S_MARGIN = 0.05


class Controller(Thread):
    def __init__(self, ref_rad_opt_flow, des_img_feature, time_keeper=time, controller=None, record='n',
                 pose_node=None):
        Thread.__init__(self)
        self._h_ref = ref_rad_opt_flow
        # NOTE: previously had a soft-engagement ramp (PLASMC_HRD_RAMP_S) and
        # a lateral-error gate (PLASMC_HRD_GATE_ALPHA) that modulated h_ref
        # during the descent. Both removed 2026-05-18: ramp gave essentially
        # zero improvement (3.07 s vs 3.29 s, within noise) and the gate
        # caused catastrophic IC 5 failure (12 m runaway) — both because
        # varying h_ref makes h_d[z] non-steady, generates dh_d transients
        # that feed the SMC c-term destructively, and shrinking h_rd also
        # shrinks the (h_rd − dot(cross(w,s), e3))·s cross-coupling on x/y
        # which alters SMC stability. Direct MATLAB-style use of h_ref
        # is the cleanest.
        # DESCENT-GATE (2026-06-22, reference-governor): RE-ATTEMPT of the above with
        # the failure mode fixed. The 05-18 gate failed because varying h_ref made h_d[z]
        # non-steady -> dh_d transients -> c-term destabilization. Fix = RATE-LIMIT the
        # gate (1-pole LPF, tau) so d(h_ref)/dt is tiny -> no dh_d spike, while still
        # slowing descent when off-center. Goal: null the lateral offset BEFORE Z (and
        # the FoV footprint ~0.89*Z) shrinks, so the marker never reaches the edge ->
        # corners stay decoded -> lateral flow stays honest (GT-verified honest at
        # altitude, corrupted <0.8m). g_min floor avoids permanent hover. Scale-free
        # (gates on normalized |s_e_n|). Default-OFF.
        self._descent_gate = os.environ.get("PLASMC_DESCENT_GATE", "0") == "1"
        self._dgate_slo  = float(os.environ.get("PLASMC_DGATE_SLO",  "0.4"))   # |s_e_n| <= slo -> g=1 (full descent)
        self._dgate_shi  = float(os.environ.get("PLASMC_DGATE_SHI",  "0.8"))   # |s_e_n| >= shi -> g=g_min (engages before the FoV edge 1.0)
        self._dgate_gmin = float(os.environ.get("PLASMC_DGATE_GMIN", "0.15"))  # descent floor (avoid permanent hover)
        self._dgate_tau  = float(os.environ.get("PLASMC_DGATE_TAU",  "0.5"))   # LPF tau on g -> rate-limit -> no dh_d transient (the 05-18 failure)
        self._dgate_g    = 1.0                                                 # filtered gate state

        self._CONTROLLER_READY = False
        self._warmup_remaining = 0           # set by startController()
        self._STAY_OPEN = True
        self.TARGET_IS_VISIBLE = False

        self._FC = controller
        self._img_node = ID.IMG_PROCESSOR(time_keeper=time_keeper, controller=controller)
        if record != 'n' or os.environ.get("IMG_RECORD", "0") == "1":
            self._img_node.RECORD = True
            print("Starting with recording...")
        else:
            print("Starting without recording...!")

        self._time = time_keeper
        self._s_d = des_img_feature

        # GT-FEEDBACK scaffold (PLASMC_GT_FEEDBACK=1): replace perception s/h with the
        # exact V-frame ground truth from the Gazebo poses, to isolate CONTROL from
        # PERCEPTION. See src/gt_feedback.py + tools/validate_gt_feedback.py.
        self._pose_node = pose_node
        self._gt_feedback = None
        if os.environ.get("PLASMC_GT_FEEDBACK", "0") == "1":
            if pose_node is None:
                print("[controller] PLASMC_GT_FEEDBACK=1 but no pose_node passed — DISABLED")
            else:
                from gt_feedback import GTFeedback
                self._gt_feedback = GTFeedback()
                # PER-CHANNEL GT ABLATION (2026-07-04): GT_ABLATE = comma-list of channels to take
                # from GT, REST from perception — isolate the binding perception signal one at a time.
                # Channels: s (centroid xy), h (lateral flow xy), hz (loom), yaw (alpha), wz (yaw rate).
                # Unset/empty/'all' -> full GT-FB (back-compat). e.g. GT_ABLATE=h -> only flow from GT.
                _abl = os.environ.get("GT_ABLATE", "all").strip().lower()
                self._gt_ablate = set() if _abl in ("all", "") else set(c.strip() for c in _abl.split(","))
                print(f"[controller] PLASMC_GT_FEEDBACK=1 — GT channels: {_abl} (perception for the rest)")

        # ---------------- MATLAB-aligned gains ----------------
        # Normalized pixel-error half-range (MATLAB: K_ctrl.p_10 = [res(2)/2/f; res(1)/2/f])
        # For Gazebo 1280x960 @ f=540: ~[1.185, 0.889]
        self._p_10 = self._img_node.center / self._img_node.focal  # (2,)

        # ════ Outer-loop Virtual Image Point PID  [manuscript: K_rp, K_ri, K_rd] ════
        # DIRECT per-axis control parameters (2026-06-03 cleanup: scale factors on
        # hidden base values replaced by the parameter values themselves; the K_rp
        # gain scheduler and ds_d clamps were removed as obsolete — see
        # parameter_record.ods sheet "Removed_Parameters" for the full history).
        #
        # Manuscript (MATLAB) values: K_rp=diag(9,9), K_ri=diag(0.1,0.1),
        # K_rd=diag(1.4375,1.4375). PX4 default K_ri=1.0 (10× MATLAB — SITL
        # LK-centroid drift correction, see docs/CONTROLLER_PARITY.md §3).
        #
        # Axis convention: image axis 0 (V-frame x) → PITCH/North,
        #                  image axis 1 (V-frame y) → ROLL/East.
        # The two axes run at DIFFERENT effective loop gains for the same physical
        # error (p_10 norm [0.889,1.185] × cal_s [1.099,1.056] → x is 1.39× hotter
        # than y) — per-axis values are how that gets compensated.
        self._K_rp = np.diag([float(os.environ.get("PLASMC_KP_X", "3.0")),
                              float(os.environ.get("PLASMC_KP_Y", "3.0"))])    # 5->3 (2026-06-14 lateral-overshoot bake): KP sets the saturated-barrier demand ceiling (∝p_s·KP); the demand is what builds the un-brakeable lateral velocity that overshoots p_1 and flies the marker out of FoV. KP=3 is the gains sweet spot at IC2 (n=5: arrival vel 3.4→2.8, peak vel 8.1→3.8, xy 22→8.6, demand h_d 2.16→1.27) with convergence intact; KP=2 goes sluggish (flt 24s). IC2-5 gate (n=3): drift down + IC5 catastrophe FIXED (27m→2.3m), no systematic regression. CAVEAT: still TL-dominated (no soft/precise) — the residual is the inner-loop perception wall (flow under-reports velocity ~2.4x so the commanded brake is never delivered; NOT gain-tunable — PS0/KD/escalating-recovery all confirmed no-help). Drift-reduction deliverable, NOT a solution.
        self._K_ri = np.diag([float(os.environ.get("PLASMC_KI_X", "0.1")),
                              float(os.environ.get("PLASMC_KI_Y", "0.1"))])   # KI 1.0->0.1 (2026-06-11 IC=2 gain-chain bake): MATLAB parity; the 10x integral was windup fuel once the P-path works — pushed ds_d=+1.4 THROUGH the crossing
        self._K_rd = np.diag([float(os.environ.get("PLASMC_KD_X", "0.5")),
                              float(os.environ.get("PLASMC_KD_Y", "0.5"))])    # K_rd 0->0.5 (2026-06-11 IC=2 gain-chain bake): MATLAB parity 0.5031 — the only phase-LEAD element; flips the demand before the crossing (overshoot 5x smaller, first damped ring-down). The old close-range D-spike concern is guarded by DH_D_MAX.
        # LPF time-constant on ds_d (the outer-PID output that feeds h_d).
        # Control runs at 125 Hz; images arrive at ~42 Hz. Each new frame causes
        # a discrete jump in s_e_n -> step in ds_d -> raw dh_d spike ~17 m/s³ at
        # K_rp=12. The EMA smooths this: at tau=50ms raw p50 11→2 m/s³, clipping
        # 64%→32%. BUT n=5 (2026-06-09): median 6.41m WORSE than 3.80m baseline.
        # 50ms lag delays outer PID correction of lateral drift → s_e_n grows faster
        # to funnel breach. Flow underreport is the binding constraint; LPF can't help
        # once s_e_n is large. Default ON (0.05s); set PLASMC_TAU_DS=0 to disable.
        self._tau_ds = float(os.environ.get("PLASMC_TAU_DS", "0.05"))

        # ════ Middle-loop control parameters — DIRECT per-axis values ════
        # Manuscript symbol mapping (docs/CONTROLLER_PARITY.md): XI2=Ξ₂, P20=p₂₀,
        # P2INF=p₂∞, OMEGA=𝒳 (PI surface integrator), GAMMA=Γ (surface
        # proportional), E=ℰ (boundary layer), N=𝒩 (adaptive growth),
        # P=𝒫 (adaptive leakage), KAPPA0=κ(0).
        # PX4 defaults differing from MATLAB (docs/CONTROLLER_PARITY.md §3):
        # N_z 0.02 (vs 0.05), P_z 2.5 (vs 5.0), kappa_0 = 1.25× MATLAB.
        def pa(key, dx, dy, dz):
            """Per-axis direct parameter PLASMC_<KEY>_{X,Y,Z}; defaults given."""
            return np.array([float(os.environ.get(f"PLASMC_{key}_X", str(dx))),
                             float(os.environ.get(f"PLASMC_{key}_Y", str(dy))),
                             float(os.environ.get(f"PLASMC_{key}_Z", str(dz)))])

        # Optic-flow funnel (LOAD-BEARING: p_2_0, p_2_∞).
        # WARNING: funnel width IS the barrier gain (G⁻¹ ≈ p/2) — never widen a
        # funnel component to "make room" for a transient; it raises that axis's
        # gain proportionally (lesson learned twice: batches 6 and 11).
        self._gamma = np.diag(pa("XI2",   0.6, 0.6, 1.0))   # XI2_z 0.8->1.0 (2026-06-26 user bake): tighter loom funnel -> lower h_e_z / better descent-rate tracking on the kappa_0_xy base (n=2 IC4, PROVISIONAL — validate IC2-5 n>=5). Earlier XI2_z=1.0 bang-bang was the FROZEN-kappa base (z-chatter coupled into the un-converged lateral); de-coupled once kappa_0_xy converges s_e_n. Takes effect when XI2_xy are env-pinned (bypasses the combined auto-align, line ~276, left coherent).
        self._p_0   =         pa("P20",   15.0, 15.0, 10.0)  # P20_x/y 25->15 BAKED 2026-06-29 (terminal-approach); P20_z 4->10 (2026-06-13)
        self._p_inf =         pa("P2INF", 1.5, 1.5, 0.5)   # P2INF_xy 2.5->1.5; P2INF_z 1.5->0.5 (2026-06-13): tighter z funnel floor -> tighter h_e_z -> softest touchdown (vel 0.37 m/s); binds because XI2_z=0.6 contracts the funnel
        # Outer-loop POSITION funnel on s_e_n (PPC, mirrors the velocity funnel above).
        # Env-gated, default OFF = the legacy outer PID. Back-mapped form is a drop-in for the
        # PID at small error (G_s^-1·zeta_dot -> -K_rp·s_e_n); the barrier bites as s_e_n -> p_s.
        self._sen_funnel = os.environ.get("PLASMC_SEN_FUNNEL", "1") == "1"   # default ON — all PX4_NewCal_Record trials ran with SEN_FUNNEL=1
        self._gamma_s = np.diag([float(os.environ.get("PLASMC_XIS_X", "0.5")),
                                 float(os.environ.get("PLASMC_XIS_Y", "0.5"))])    # gamma_s 1.0->0.5 (2026-06-11 IC=2 gain-chain bake): at 1.0 the funnel OVERTAKES a large-IC error at t~1.2 → ratio saturation → G_s⁻¹∝p_s collapse → DEMAND STARVATION; 0.5 keeps the funnel valid (sustained demand, genuine closure)
        # Outer position-funnel start, RESOLUTION-DERIVED (2026-06-15). The barrier runs
        # on s_e_n = s_e / p_10 (p_10 = center/focal = half-FoV per axis), so the FoV edge
        # is |s_e_n| = p_10/p_10 = 1.0 on BOTH axes (resolution is absorbed by p_10). The
        # log-barrier needs p_s > |s_e_n|, so p_s_0 = FoV-edge + a validity margin keeps it
        # valid for ANY in-FoV start (|s_e_n| up to 1.0). Default margin 0.2 → p_s_0 = 1.2
        # (was a hardcoded 1.2 that the old "≈FoV edge 1.185" comment mis-derived by
        # double-counting the p_10 normalization). PLASMC_PS0_MARGIN sets the margin;
        # PLASMC_PS0_X/Y still override the final value outright.
        _fov_edge_sen = self._p_10 / self._p_10                       # = [1,1]; FoV boundary in s_e_n units
        _ps0_margin   = float(os.environ.get("PLASMC_PS0_MARGIN", "0.2"))
        _ps0_def      = _fov_edge_sen * (1.0 + _ps0_margin)          # edge + barrier-validity margin
        self._p_s_0   = np.array([float(os.environ.get("PLASMC_PS0_X", str(_ps0_def[0]))),
                                  float(os.environ.get("PLASMC_PS0_Y", str(_ps0_def[1])))])
        self._p_s_inf = np.array([float(os.environ.get("PLASMC_PSINF_X", "0.35")),
                                  float(os.environ.get("PLASMC_PSINF_Y", "0.35"))])   # p_s_inf 0.1->0.35 (2026-06-11 IC=2 gain-chain bake): the funnel FLOOR must exceed the damped post-crossing swing or the barrier saturates exactly when needed; angular bound — at Z=0.2m, 0.35 still = ~0.07m metric
        # Optic-flow ASMC (LOAD-BEARING: OMEGA/𝒳)
        self._Omega = np.diag(pa("OMEGA", 0.1, 0.1, 0.1))     # OMEGA_z 0.025->0.1 (2026-06-13 user bake, Ez5_combo) — more z integral action
        self._Gma   = np.diag(pa("GAMMA", 2.0, 2.0, 1.0))       # GAMMA_xy 0.4375/0.5->2.0 + GAMMA_z 0.75->1.0 (2026-06-13 soft-config bake): reaching gain — the un-saturated proportional term that actually speeds h_e reaching (κ_0/E are sat-gated); xy=2.0 uses full w_u authority, sharper lateral convergence
        self._E     = np.diag(pa("E",     0.8, 0.8, 0.5))       # E_z 0.1->0.5 (2026-06-13 user bake, Ez5_combo) — WIDER z boundary layer; tames the z over-brake (κ_z held ~1.3, au_z ~41 vs the cap/122 at E_z=0.1+aggressive funnel). ⚠️ Ez5_combo (this 4-param set) was CATASTROPHIC at IC2 n=5 (4/5 TL, mean 26.5) — but the failure was LATERAL (z-loop tame); kept as a non-over-braking z baseline for lateral work. Soft-config was E_z=0.1 (vel 1.3-1.8)
        # Adaptive-gain ODE
        self._N       = np.diag(pa("N",      0.1, 0.1, 0.1))   # N_xy 0.02->0.1 BAKED 2026-06-25 (user): REMOVES the frozen-kappa_xy band-aid (N=0.02 -> tau=1/(N*P)=33s >> 7s descent -> kappa_xy never adapts AND inadvertently CAPS the terminal switching by being too slow to ramp). N=0.1 (tau~7s) wakes kappa_xy -> tighter descent s_e_n BUT exposes the terminal: at the deck ζ_h explodes -> the fast κ-ODE ramps κ_xy 0.42->13.5 -> a_u_xy 64x -> launch -> REGRESSES tally 7/0/1 -> 4/2/3 (gate 175559 vs 145330). The terminal is the actuator wall ([[feedback_terminal_smc_actuator_wall]]) -> fix via velocity damping (flow funnel / arrest v_lat early), NOT the frozen kappa. P_xy=1.5 (leakage) bounds kappa_eq=θG|σ|/P. N_z=0.1: a LEGITIMATE PX4-specific divergence from MATLAB vdf_params (P.N_z=0.02), NOT a stale bug. VDF-parity A/B 2026-06-22 (Nz_IC2, N=15, clean combined baseline) REJECTED N_z=0.02: it RAISED terminal descent |vz| spikes (vzmax med 3.57→5.10, p90 6.56→9.59) + regressed sub-meter 5→3/15 + TL 1→3. Reason: PX4's SITL descent is only ~2s so κ_z is ADAPTATION-RATE-LIMITED — faster N_z=0.1 ramps κ_z up in time for terminal z-BRAKING; the slow VDF 0.02 under-brakes (MATLAB's slower dynamics make 0.02 right THERE). N = κ-ODE adapt rate (dκ/dt=Θ·N·G·|σ|−N·P·κ). Like chi_r=0.5 vs 0.85, a SITL-timing divergence. Do NOT re-align to 0.02.
        self._P       = np.diag(pa("P",      1.5, 1.5, 5.0))     # P_xy 5->1.5 (2026-06-12 Combo bake): MATLAB parity; paired with KP=5 the lateral kappa stays bounded without the over-gained P=5
        self._kappa_0 =         pa("KAPPA0", 0.5, 0.5, 1.0)     # KAPPA0_xy 0.125->0.5 (gain-chain crossing brake); KAPPA0_Z 0.25->1.0 (2026-06-13 soft-config bake): the BOOTSTRAP value — z braking authority from t=0 -> soft touchdown (vel 4.4->1.3-1.8 m/s) AND prevents the E_z=0.1 κ_z ratchet
        self._kappa_max = pa("KAPPA_MAX", 1e6, 1e6, 3.0)                # KAPPA_MAX_Z=3.0 (REVERTED from 10.0, 2026-06-13): the IC2-5 gate CONFIRMED 10.0 is net-negative — κ_z ran to 10 in the drift/hard reps (IC3_rep4 11.5 m/s, IC4) where 3.0 would have held it (more violent), while clean soft reps sit at κ_z~1 (cap irrelevant). The 3.0 backstop is load-bearing in bad reps, inert in good ones.
        self._dw_max    = float(os.environ.get("PLASMC_DW_MAX", "30.0"))   # physical clamp on |dw| (rad/s²) for the c-term feedforward
        # Cap on the |omega_dot x s| c-term sub-term (the angular-accel feedforward). 0 = OFF (no cap).
        # dw is already clamped to dw_max, but omega_dot x s still explodes when the centroid s is LARGE
        # at a terminal funnel breach (s_e_n~1) -> it became the DOMINANT a_u_xy driver (-121 of -146 at
        # the IC1 32m launch; switching was tamed to +-4 by the kappa-cap + per-axis theta). This caps
        # that ungated feedforward so it can't saturate the tilt/thrust and launch the drone. Magnitude
        # cap (direction preserved); small enough to bite the terminal spike, large enough to leave the
        # normal-flight FF intact.
        self._cterm_dws_max = float(os.environ.get("PLASMC_CTERM_DWS_MAX", "0"))
        if self._cterm_dws_max > 0:
            print(f"[PLASMC] PLASMC_CTERM_DWS_MAX={self._cterm_dws_max} — capping |omega_dot x s| c-term feedforward")
        # Scale on the loom×flow c-term feedforward -(h·e3)h (=+|h_z|·h). 2026-06-26 decomposition:
        # this term is the OUTWARD driver of the mid-descent s_e_n hump (radial +0.77 vs the switching's
        # -0.64) — a loom×flow POSITIVE feedback (lat drift -> flow h_xy -> term amplifies -> more drift).
        # =1.0 keeps it (the honest known-dynamics FF); <1.0 gates the amplifier to kill the hump.
        self._cterm_loom_scale = float(os.environ.get("PLASMC_CTERM_LOOM_SCALE", "1.0"))
        if self._cterm_loom_scale != 1.0:
            print(f"[PLASMC] PLASMC_CTERM_LOOM_SCALE={self._cterm_loom_scale} — scaling the -(h·e3)h loom×flow c-term feedforward")
        # PER-AXIS theta decoupling (BAKED default-ON 2026-06-25). The shared scalar theta=||Theta||_F
        # couples the axes: the LATERAL position-barrier (zeta_r) terminal explosion inflates theta ->
        # detonates the switching term + kappa-ODE on EVERY axis incl z (the z over-brake is COLLATERAL,
        # see feedback_terminal_root_lateral_zeta_r). Per-axis theta_i = sqrt(vector_i^2 + 1) (row-i norm
        # of Theta=[vector|I3]) gives each axis its OWN uncertainty bound; sqrt(sum theta_i^2)==||F|| so
        # it's an EXACT decomposition (UUB-PRESERVED, tight bound -- proof Drafts/PER_AXIS_THETA_PROOF.md).
        # n=3 GT-FB IC1-5: 12/15 sub-meter, 0 stalls, z DECOUPLED (terminal a_u_z bounded ~4 while lateral
        # a_u_xy explodes). The 2/15 fly-aways are the un-fixed LATERAL zeta_r/s_e_n root (NOT this change
        # -- a_u_z stayed 4.0 in both flys), the next lever. Set PLASMC_THETA_PER_AXIS=0 for legacy scalar.
        self._theta_per_axis = os.environ.get("PLASMC_THETA_PER_AXIS", "1") == "1"
        if self._theta_per_axis:
            print("[PLASMC] per-axis theta ON (decoupled switching + kappa-ODE; PLASMC_THETA_PER_AXIS=0 for legacy scalar)")

        # ════ COMBINED-BARRIER sliding surface (manuscript combined surface; default OFF) ════
        # Aligned to the canonical MATLAB realization (visualControl_IBVS_adaptive.m, a152479).
        # Replaces the SEN back-map: the position barrier zeta_r enters sigma DIRECTLY
        # (lateral sigma_k = zeta_h_k + chi_r*zeta_r_k), so there is no G_s^-1->0 demand
        # starvation (the lateral-wall / IC5 deficit). h_d uses the MEASURED centroid rate
        # s_dot (no back-mapped ds_d); s_ddot is dropped from dh_d (kappa absorbs it as d_h).
        # zeta_r is built on r_bar_e = s_e/p_10 (= s_e_n, FoV-normalized) with its OWN funnel
        # p_r (FoV-consistent floor p_r_inf>=1 — proof Standing Condition 1; precision comes
        # from p_2/chi_r, NOT from tightening p_r). MATLAB-validated 25/25 SP + 75/75 noisy.
        self._combined_barrier = os.environ.get("PLASMC_COMBINED_BARRIER", "1") == "1"  # RE-BAKED 2026-06-20 with MANUSCRIPT gains (the earlier regress was a gain-parity bug; vdf_params auto-applied)
        self._chi_r   = np.array([float(os.environ.get("PLASMC_CHI_R_X", "1.5")),
                                  float(os.environ.get("PLASMC_CHI_R_Y", "1.5"))])   # BAKED 1.5 2026-06-29 (terminal-approach config; was 0.5)   # BAKED 0.5 (2026-06-20): PX4 LATERAL-VELOCITY-ARREST tuning. surface PD balance sigma=zeta_h+chi_r*zeta_r; LOWER chi_r weights the velocity/damping term (zeta_h) more -> less overshoot -> lower terminal lateral velocity (IC2: vlat 2.61->1.45, xy 1.04->0.43). ⚠️ DIVERGES from MATLAB manuscript 0.85 (max-margin manifold) -- PX4-specific because the SITL flow lag adds overshoot the noiseless MATLAB lacks. Env-overridable.
        self._p_r_0   = np.array([float(os.environ.get("PLASMC_PR0_X", "10.0")),
                                  float(os.environ.get("PLASMC_PR0_Y", "10.0"))])     # position-funnel initial half-width (FoV units). BAKED 10.0 2026-06-29 (was 1.2): the FUNNEL-SHAPE fix. A WIDE initial funnel keeps S_r=s_e_n/p_r tiny (~0.05) the whole descent so zeta_r never enters the steep barrier edge -> no edge-forcing, no terminal 1/Z balloon. With slow XIR=0.10 the funnel decays 10->~4 (never reaches the floor -> PRINF inert), so s_e_n converges + STAYS converged. GT-FB IC1-5: clean (0 breach), xy~0.13-0.22, IC5 canary solved. The terminal funnel width p_r(T)=PRINF+(10-PRINF)e^{-XIR*T} absorbs the terminal 1/Z spike (wide -> S_r stays <1). REQUIRES slow XIR (fast XIR collapses the funnel before s_e_n converges).
        self._p_r_inf = np.array([float(os.environ.get("PLASMC_PRINF_X", "0.8")),
                                  float(os.environ.get("PLASMC_PRINF_Y", "0.8"))])   # BAKED 0.8 2026-06-29 (terminal-approach; was 1.0)    # terminal floor — FoV-consistent (>=1 keeps the CBF->funnel transfer exact)
        self._xi_r    = np.diag([float(os.environ.get("PLASMC_XIR_X", "0.10")),
                                 float(os.environ.get("PLASMC_XIR_Y", "0.10"))])      # position-funnel contraction rate. REVERTED 0.15->0.10 2026-07-01: the 486f713 bake to 0.15 was validated at P2INF_xy=1.5, but at the BAKED P2INF_xy=1.0 it REGRESSES. Controlled same-binary A/B (212524 vs 220308, soft-breach OFF both, only XIR differs): XIR=0.15 -> 12/25 SP vs XIR=0.10 -> 19/25 SP (IC1-5 n=5, GT-FB). Gain concentrated in OFF-CENTER IC2 (1->5) & IC3 (2->5) = the funnel-contraction mechanism (slow XIR keeps p_r wide -> off-center s_e_n converges & STAYS converged; fast XIR contracts onto the off-center error -> edge-forces). IC5 4->2 = the predicted short-runway tradeoff (net +7). XIR>=0.20 collapses the funnel before s_e_n converges.
        # Combined mode = the MATLAB VDF-ASMC manuscript controller. The baked PX4 defaults
        # above are the BACK-MAPPED soft-config gains (GAMMA=2.0, KAPPA0=0.5, XI2=0.6, E=0.8/0.5,
        # P2INF_z=0.5) which are 3-5x too HOT for the combined surface (zeta_r already supplies
        # position authority) -> a_u over-aggression (~102 m/s2) -> off-center fly-aways. Auto-align
        # to MATLAB/VDF_ASMC/vdf_params() (manuscript-validated 25/25 SP) in combined mode, each
        # only if not explicitly overridden by env.  2026-06-20 parity fix.
        if self._combined_barrier:
            if "PLASMC_P2INF_X" not in os.environ: self._p_inf[0] = 1.0   # KEPT 1.0 2026-06-30: A/B at {h_rd=-0.30,XIR=0.15} (IC2/IC4/IC5 n=3) REVERSED the old 1.5 claim — P2INF=1.0 best (7/9 SP, rel_vel med 0.026, 0 fly) vs 1.5 (5/9, 0.144, 1 fly) vs 2.0 (5/9, 0.172). The old "1.5 un-saturates zeta_h" benefit is LOST at this config (h_e spikes saturate zeta_h even at 1.5); smaller p_2 = steeper zeta_h = more velocity damping at the terminal. (Briefly baked 1.5 then reverted.)
            if "PLASMC_P2INF_Y" not in os.environ: self._p_inf[1] = 1.0   # KEPT 1.0 2026-06-30 (A/B: 1.0 > 1.5/2.0)
            if "PLASMC_P2INF_Z" not in os.environ: self._p_inf[2] = 1.5      # vdf p_hinf z
            if not any(f"PLASMC_GAMMA_{a}" in os.environ for a in "XYZ"):
                self._Gma = np.diag([0.25, 0.25, 0.75])                      # GAMMA_xy 0.4375/0.5->0.25 BAKED 2026-06-29 (symmetric): reaching gain a_u=-Gamma*sigma is the terminal-limit-cycle FORCING amplitude (sigma rings in the boundary layer terminally); lower Gamma shrinks the cycle -> softer + more precise. GT-FB sweep {0.25,0.5,1.0} @ PR0=10/PRINF=0.8/XIR=0.10: 0.25 best (xy 0.087 3/4 precise, rel_vel 0.38, vlat_term 0.23 vs 0.31/0.63). Symmetric (GT-FB has no hot axis; x 1.39x is a perception/cal asymmetry -> re-check per-axis under perception-ON). Z=0.75 (VDF, descent) unchanged. (was vdf 0.4375/0.5; pre-vdf bare default 2/2/1)
            if not any(f"PLASMC_KAPPA0_{a}" in os.environ for a in "XYZ"):
                self._kappa_0 = np.array([0.5, 0.5, 0.25])                  # BAKED 2026-06-29 terminal-approach kappa0 (was 0.125/0.125/0.25 VDF)
            if not any(f"PLASMC_E_{a}" in os.environ for a in "XYZ"):
                self._E = np.diag([1.0, 1.0, 0.5])                          # vdf E; E_z 1.0->0.5 BAKED 2026-06-21 (IC2 N=15 x2: xy std 29.5->~4, fly 5/15->2-4/15, TL 5/15->1/15; engages kappa switching damping on the terminal Z cycle per MATLAB CB57). X/Y stay 1.0 (NOISE-pumped, kappa hurts there). Env PLASMC_E_* still overrides.
            if not any(f"PLASMC_XI2_{a}" in os.environ for a in "XYZ"):
                self._gamma = np.diag([0.7, 0.7, 1.0])                      # BAKED 2026-06-29 terminal-approach Xi_h (was 0.2/0.2/0.2 VDF)
            # (self._kappa is seeded from self._kappa_0 below at its init, picks up the new value)

        # Print every parameter whose value differs from its default.
        _defaults = {"XI2": (0.2, 0.2, 0.2), "P20": (25.0, 25.0, 4.0),
                     "P2INF": (2.5, 2.5, 1.5), "OMEGA": (0.05, 0.05, 0.025),
                     "GAMMA": (0.4375, 1.0, 0.75), "E": (1.5, 1.5, 1.0),
                     "N": (0.02, 0.02, 0.02), "P": (5.0, 5.0, 5.0),
                     "KAPPA0": (0.15625, 0.15625, 0.3125)}
        _values = {"XI2": np.diag(self._gamma), "P20": self._p_0, "P2INF": self._p_inf,
                   "OMEGA": np.diag(self._Omega), "GAMMA": np.diag(self._Gma),
                   "E": np.diag(self._E), "N": np.diag(self._N), "P": np.diag(self._P),
                   "KAPPA0": self._kappa_0}
        _print_lines = []
        for k, dflt in _defaults.items():
            for i, ax in enumerate(("X", "Y", "Z")):
                if abs(_values[k][i] - dflt[i]) > 1e-12:
                    _print_lines.append(f"  {k}_{ax} = {_values[k][i]:g}  (default {dflt[i]:g})")
        # Log FoV / funnel env vars that can silently override behaviour and are NOT
        # covered by the per-axis checker above (e.g. THETA_FLOOR_DEG).
        _fov_vars = [
            ("PLASMC_THETACAP_DEG",  "60.0"),
            ("PLASMC_THETA_FLOOR_DEG", "60.0"),
            ("FLOW_FUSE_RING",       "1"),
            ("PLASMC_SEN_FUNNEL",    "1"),
            ("BODY_YAW_SOURCE",      "alpha"),
            ("PLASMC_TAU_DS",        "0.05"),
            ("PLASMC_DSD_LAT_MAX",   "100.0"),
            ("PLASMC_SEN_RECOVERY_K", "0.0"),
            ("CBF_LPF_BEFORE",       "0"),
            ("FLOW_CENTROID_RATE",   "0.0"),
            ("PLASMC_CH_CLEAN",      "1"),
        ]
        for _var, _dflt in _fov_vars:
            _val = os.environ.get(_var, _dflt)
            if _val != _dflt:
                _print_lines.append(f"  {_var} = {_val}  (default {_dflt})")
        if _print_lines:
            print("[PLASMC] non-default parameters:")
            for line in _print_lines: print(line)

        # ════ Yaw ASMC  [manuscript: χ_α, γ_α, η_α, ρ_α, κ_α(0), ε_α] ════
        # (ROBUST class per supplement S3-A — ±50% moves metrics <0.5cm in MATLAB;
        # in PX4 the yaw chain has 287ms lag so values ~0.2 are used — memory
        # convergence-ordering.)
        # YAW Omega_a 0.5->0.1 BAKED 2026-06-23 (GT-feedback per-axis study, NC116-121):
        # u_a is a yaw-RATE command (psi_d=int(u_a)), so the rate structure already
        # integrates e_a; the Omega_a*ie_a term added a SECOND integrator -> no phase
        # margin vs the PX4 inner-loop lag (K_R_YAW + rate loop + tau_ua LPF, absent in
        # MATLAB) -> growing yaw limit cycle that pumps lateral via image-frame coupling.
        # 0.1 removes the double-integrator: yaw cycle eliminated (ncross 5-6->2, single
        # bounded overshoot) across DES_ALPHA 0/10/30/45 at IC1, no fly-aways. A PX4-lag
        # divergence (cf chi_r=0.5, N_z=0.1). RULED OUT: K_R_YAW^ (worse), E_a v (no help
        # at Omega_a=0.5). Env PLASMC_YAW_OMEGA still overrides.
        self._Omega_a   = float(os.environ.get("PLASMC_YAW_OMEGA",  "0.1"))
        self._Gma_a     = float(os.environ.get("PLASMC_YAW_GAMMA",  "0.5"))
        self._n_a       = float(os.environ.get("PLASMC_YAW_N",      "1.0"))
        self._p_a       = float(os.environ.get("PLASMC_YAW_P",      "2.0"))
        self._kappa_a_0 = float(os.environ.get("PLASMC_YAW_KAPPA0", "2.0"))
        self._E_a       = float(os.environ.get("PLASMC_YAW_E",      "3.0"))
        # YAW ALPHA SAVGOL-PREDICTION FILTER (2026-06-20): reject the terminal alpha (s[3])
        # corruption spike (single-frame -22° outlier at touchdown when the marker fills the FoV).
        # Fit a low-order Savgol/poly to the recent alpha history -> a smooth PREDICTION; if the raw
        # alpha deviates from it by > YAW_ALPHA_REJECT rad, use the prediction (KLT-style outlier
        # reject), else blend toward raw. Tracks genuine slow yaw, kills the spike. Default-ON.
        self._yaw_alpha_filt = os.environ.get("PLASMC_YAW_ALPHA_FILT", "1") == "1"
        self._yaw_alpha_win = int(os.environ.get("PLASMC_YAW_ALPHA_WIN", "9"))    # savgol window (frames)
        self._yaw_alpha_max_rate = float(os.environ.get("PLASMC_YAW_ALPHA_MAX_RATE", "0.30"))  # rad/s (~17°/s) drift cap (genuine yaw ~0.04 rad/s, corruption ~1 rad/s)
        self._alpha_hist = deque(maxlen=self._yaw_alpha_win)   # recent (unwrapped) raw alpha
        self._alpha_smooth = None      # last smoothed/predicted alpha (unwrapped)
        # ALPHA CV-KF (2026-06-20): constant-velocity Kalman on the yaw angle [yaw, yaw_rate] with
        # INNOVATION GATING — the principled alternative to rate-limit+Savgol. Models the yaw
        # dynamics (predicts via the estimated rate), and a measurement inconsistent with the
        # prediction (the corruption drift) produces a large innovation that the Mahalanobis gate
        # down-weights/rejects. PLASMC_YAW_ALPHA_KF=1 selects it over the Savgol path (both gated
        # by PLASMC_YAW_ALPHA_FILT). Genuine slow yaw passes (small innovation). Default-off (A/B).
        self._yaw_alpha_kf = os.environ.get("PLASMC_YAW_ALPHA_KF", "0") == "1"
        self._yaw_kf_q = float(os.environ.get("PLASMC_YAW_KF_Q", "5.0"))     # process (yaw-accel) noise
        self._yaw_kf_r = float(os.environ.get("PLASMC_YAW_KF_R", "1e-3"))    # measurement (alpha) noise var
        self._yaw_kf_gate = float(os.environ.get("PLASMC_YAW_KF_GATE", "9.0"))  # innovation² gate (χ², ~3σ)
        self._yaw_kf_x = None          # [yaw, yaw_rate] state (yaw unwrapped)
        self._yaw_kf_P = None          # 2×2 covariance

        # ════ FoV-margin cone clamp  [manuscript: p₁₀, p₁∞, ξ₁, θ_cap] ════
        # Pixel envelopes per image axis (U/V), DIRECT values in px.
        # Defaults = 2× MATLAB (camera is 640×480 @ f=270 vs 320×240 @ f=135).
        # In BOARD mode these protect the PRIMARY marker's corners only (see
        # docs/CONTROLLER_PARITY.md board-mode note). The validated IC1 config sizes
        # the envelope to the sensor edge: U0=290, V0=315, Uinf=220, Vinf=300.
        self._rho_fov_0   = np.array([float(os.environ.get("PLASMC_RHOFOV0_U",   "290.0")),
                                      float(os.environ.get("PLASMC_RHOFOV0_V",   "210.0"))])
        self._rho_fov_inf = np.array([float(os.environ.get("PLASMC_RHOFOVINF_U", "80.0")),
                                      float(os.environ.get("PLASMC_RHOFOVINF_V", "80.0"))])
        # rho_fov held CONSTANT at rho_fov_0 by default (l_fov=0 -> exp(0)=1 -> rho_fov_curr=rho_fov_0).
        # The decay to rho_fov_inf (80px) shrank the visibility funnel far inside the camera FoV,
        # firing the perception-death handoff prematurely (marker fills 80px while still visible to
        # ~290px). Constant rho_fov_0 = a fixed near-camera-FoV visibility limit; precision/convergence
        # is the SMC's job, not the visibility funnel's. Set PLASMC_LFOV>0 to restore the decay. (2026-06-05)
        self._l_fov     = float(os.environ.get("PLASMC_LFOV", "0.0"))
        self._theta_cap   = np.deg2rad(float(os.environ.get("PLASMC_THETACAP_DEG", "60.0")))
        self._theta_floor = np.deg2rad(float(os.environ.get("PLASMC_THETA_FLOOR_DEG", "60.0")))
        self._DH_D_MAX    = float(os.environ.get("PLASMC_DH_D_MAX", "50.0"))
        # Lateral approach-velocity governor (2026-06-14). The outer PID demand
        # ds_d[xy] (= desired lateral optical-flow, scale-free v/Z) is pinned at
        # the saturated-barrier ceiling ~3 for the whole approach -> commands a
        # closing flow the drone can't see (LK under-reports) -> arrives at the
        # target carrying ~4-5 m/s lateral -> flies through -> fly-away (the
        # lateral wall, see feedback_lateral_overshoot_root). Capping |ds_d[xy]|
        # gentles the closing demand so the velocity stays in the flow-observable
        # range -- the depth-free analog of the descent h_rd governor. Caps the
        # feature-rate magnitude only (no Z/metric). Default 100 = effectively off.
        self._DSD_LAT_MAX = float(os.environ.get("PLASMC_DSD_LAT_MAX", "100.0"))
        # Escalating recovery authority on outer-funnel breach (see _updateImgFeatureParam).
        # Gain on the inward velocity demand added when |s_e_n| > p_s, proportional to the
        # overflow. Restores the recovery authority the back-mapped barrier collapses + the
        # ratio clamp freezes. Default 0.0 = OFF.
        self._sen_recovery_k = float(os.environ.get("PLASMC_SEN_RECOVERY_K", "0.0"))
        # CV-KF for V_ds (combined-barrier centroid rate). Default-off (PLASMC_VDS_KF=0 → the
        # MATLAB-parity smooth4(backward finite-diff)). When ON: a constant-velocity Kalman filter
        # on s_e[:2] estimates position+velocity jointly; V_ds = the velocity state (lower lag + no
        # double-smoothing than diff-of-already-KF'd-position). NB: a PX4-side divergence from MATLAB.
        self._vds_kf = os.environ.get("PLASMC_VDS_KF", "1") == "1"   # RE-BAKED 2026-06-20 with combined (validated V_ds estimator)
        self._vds_kf_q = float(os.environ.get("PLASMC_VDS_KF_Q", "10.0"))  # process (accel) noise PSD. RE-BAKED 1.0->10.0 (2026-07-04, user): the 06-30 lowering to 1.0 was to damp the terminal 1/Z² s_dot osc, but PR0=10 (06-29 funnel-shape fix) ALREADY absorbs the terminal 1/Z (A/B: termosc stays 0.009-0.012 even at q=10). The low q's only remaining effect was DRIFT-TERM-NOISE damping that masked a DIFFERENT root: the fly-aways are a TERMINAL-DECK event (drone reaches deck clean @~0.05m, marker Ncorn->0 from 1/Z fill, drone still armed -> reacts to the perc spike -> climbs+flies; q=10 makes it worse, q=1 milder — but q is a severity band-aid, NOT the fix). q=10 restores low-lag off-center velocity; the real fix is the terminal commit/disarm. ⚠ WITHOUT that fix q=10 shows 7-31m deck fly-aways.
        self._vds_kf_r = float(os.environ.get("PLASMC_VDS_KF_R", "1e-3"))   # measurement (centroid) noise var
        # RESCALE (sensor-cal CONSISTENCY, not GT): V_ds=d(s_e)/dt is built from the centroid, which
        # carries _sensor_cal_s (~1.16x lateral), whereas the flow h it is differenced against in
        # h_e=h-h_d carries _sensor_cal_hw — a DIFFERENT cal. So V_ds and h are on mismatched scales
        # by construction (measured |V_ds|/|h|~1.26 ≈ cal_s/cal_hw), biasing h_e. Rescale V_ds onto the
        # h scale (×cal_hw/cal_s≈0.79) so h_d and h are commensurate. Applies to BOTH KF and smooth4
        # V_ds (cal mismatch is estimator-independent). Default 1.0 = off; set ~0.79.
        self._vds_kf_scale = float(os.environ.get("PLASMC_VDS_KF_SCALE", "0.79"))
        self._vds_x = None         # KF state [px,py,vx,vy]
        self._vds_P = None         # KF covariance (4x4)
        # CV-KF options for the c-term derivatives dh_d (rate of h_d_noS) and dw (rate of w_i).
        # Default-off → smooth4(finite-diff) (MATLAB parity). A CV-KF on the underlying signal
        # outputs the rate as a state (lower lag + spike-spreading vs differencing). dw especially:
        # w_i is frame-held (stair-step) so finite-diff SPIKES at frame changes; the KF spreads it.
        self._dhd_kf = os.environ.get("PLASMC_DHD_KF", "1") == "1"   # BAKED 2026-06-20 (CV-KF, spike -60% vs smooth4)
        self._dw_kf  = os.environ.get("PLASMC_DW_KF", "1") == "1"   # BAKED 2026-06-20 (CV-KF, spike -80%; w_i frame-held)
        self._deriv_kf_q = float(os.environ.get("PLASMC_DERIV_KF_Q", "10.0"))
        self._deriv_kf_r = float(os.environ.get("PLASMC_DERIV_KF_R", "1e-3"))
        self._dhd_kf_st = {"x": None, "P": None}
        self._dw_kf_st  = {"x": None, "P": None}
        # TERMINAL COMMIT (2026-06-19, feedback_lateral_wall_anti_restoring_au). The lateral wall
        # ROOT = terminal 1/Z amplification: the drone descends fine to ~0.6 m with a small residual
        # offset (~0.85 m), then s_e_n=lat/Z blows up near touchdown → funnel breach → violent lateral
        # over-reaction → marker leaves FoV → TARGET_LOST → open-loop fly-away. FIX: once the marker
        # fills the FoV (MARKER_EXTENT_PX > threshold, scale-free proxy, latched + 3-frame confirm to
        # survive the switching-noisy extent), FREEZE s_e_n at its modest pre-amplification value so
        # the lateral loop stops chasing the 1/Z blow-up and just commits to descend. Default 0 = OFF.
        self._commit_extent = float(os.environ.get("PLASMC_COMMIT_EXTENT", "0"))
        # Terminal lateral-command cap applied once committed (command-bounding, NOT freezing
        # s_e_n — the freeze lies to the controller and destabilizes, 0/3 variants). Bounds
        # |V_ds_d_xy| so the 1/Z-amplified s_e_n can't whip the drone out of FoV near touchdown,
        # while the loop still sees the live, consistent error. 0 = OFF.
        self._commit_dsd_max = float(os.environ.get("PLASMC_COMMIT_DSD_MAX", "0"))
        # Terminal a_u-cap (combined-barrier: bounds the zeta_r->a_u spike that the V_ds_d cap can't
        # reach). Caps |a_u_xy| once committed. 0 = OFF. See PLASMC() a_u computation.
        self._commit_au_max = float(os.environ.get("PLASMC_COMMIT_AU_MAX", "0"))
        # Global (all-altitude) lateral-accel cap — tames the off-center APPROACH over-aggression
        # (a_u_xy ~102 at ~2.3m), which is where 53% of combined fly-aways breach. 0 = OFF.
        self._au_max_xy = float(os.environ.get("PLASMC_AU_MAX_XY", "0"))
        # Global (all-altitude) vertical-accel cap -- bounds |a_u_z| so a stale-perception
        # reacquisition burst (h_z frozen during coast -> real error dumped at once on
        # reacquisition, seen hitting -7.4 m/s^2 in real flight 2026-08-03) can't fire an
        # aggressive one-shot climb/descent command. 0 = OFF.
        self._au_max_z = float(os.environ.get("PLASMC_AU_MAX_Z", "0.2"))
        self._commit_win = int(os.environ.get("PLASMC_COMMIT_WIN", "7"))   # median window vs extent spikes
        self._ext_win = deque(maxlen=self._commit_win)
        self._committed = False
        self._commit_count = 0
        # TERMINAL RING-COMMIT (2026-07-04, project_terminal_velocity_handover_design). Retarget of
        # TERMINAL_COMMIT: keep marker s + zeta_r LIVE (moving-target position), swap ONLY the velocity —
        # h_xy->0 (ring lateral is unobservable) + h_z->ring loom (marker-less) — gated on img
        # HANDOVER_LATCHED + centered (|s_e_n|<TC_SEN) + settled (|ds_e_n|<TC_DSEN). Does NOT zero zeta_r
        # (centered gate => zeta_r already small). Default off; A/B vs the (disabled) TERMINAL_COMMIT.
        # Co-enable PLASMC_DESCENT_GATE=1 so the handover->centered window descends cautiously.
        self._terminal_ring_commit = os.environ.get("PLASMC_TERMINAL_RING_COMMIT", "0") == "1"
        self._ring_committed = False
        # LOOM-RING-ON-LOSS (2026-07-05, user): the SIMPLE rule — when the terminal decode is lost
        # (observer inactive = Ncorn->0) BUT the marker had NOT drifted off the FoV (it OVERFLOWED /
        # was occluded while still over the target, per img _last_drifted_off), swap ONLY h_z onto the
        # marker-less ring loom (vertical brake survives the deck occlusion). h_xy untouched. Reuses the
        # existing PLASMC_MARKER_FOV_MARGIN overflow-vs-drift logic. Independent of the full ring-commit.
        self._loom_ring_on_loss = os.environ.get("PLASMC_LOOM_RING_ON_LOSS", "0") == "1"
        # TERMINAL-COMMIT LATERAL TAPER (2026-06-22): the descent-gate centers the drone to the
        # deck (commit-ready 14/15 IC2) but the final ~0.3 m destabilizes — at Z<0.3 the 1/Z-
        # corrupted lateral flow spikes a_u -> launch. Once committed (marker fills FoV at the
        # centered-low state), STOP active lateral steering: ramp a_u_xy -> COMMIT_LAT_FLOOR so
        # the controller can't react to the garbage flow; coast laterally + descend level. Stronger
        # than COMMIT_AU_MAX (a cap still pushes on bad flow). Rate-ramped. Stacks on DESCENT_GATE.
        self._commit_lat_taper = os.environ.get("PLASMC_COMMIT_LAT_TAPER", "0") == "1"
        self._commit_lat_floor = float(os.environ.get("PLASMC_COMMIT_LAT_FLOOR", "0.0"))   # a_u_xy scale floor once committed
        self._commit_ramp_s    = float(os.environ.get("PLASMC_COMMIT_RAMP_S", "0.5"))      # taper time const
        self._commit_taper_c   = 1.0
        # CENTERED-GATING (2026-06-22): the extent-only latch commits on ALTITUDE alone, so it
        # can fire while still off-center (LAT@commit up to 2.74 m IC2) -> coasts to that offset
        # -> 0/15 sub-meter. Require |s_e_n| < COMMIT_SEN (centered) AT the trigger too, so the
        # commit only fires from a centered state -> lands near center. 0 = extent-only (old).
        self._commit_sen = float(os.environ.get("PLASMC_COMMIT_SEN", "0.3"))
        self._s_e_n_hold = None
        # ───────────────────────────────────────────────────────────────────────────────
        # TERMINAL-COMMIT FRAMEWORK (2026-06-28, design TERMINAL_KICK_COMMIT_DESIGN.md).
        # The kick = residual lateral VELOCITY v_res, 1/Z-normalized -> s_e_n breach ->
        # zeta_r->inf. Fix: at the corner-exit event (marker fills FoV, MARKER_EXTENT_PX >=
        # TC_EXTENT ~= a FIXED, start-height-INDEPENDENT altitude ~0.5 m), classify and act:
        #   case (b) COMMIT  if |s_e_n| < TC_SEN (centered) AND h_z < 0 (genuine loom) AND
        #                    |ds_e_n| < TC_DSEN (settled — catches the zero-crossing the
        #                    position-only gate misses), held TC_FRAMES consecutive frames.
        #                    Action: ZERO zeta_r in the lateral surface (sigma_xy -> zeta_h
        #                    only) — removes the zeta_r blow-up term -> no kick. One-way latch.
        #                    (Honest: marker invisible <0.3 m, so s_e_n is fiction.) Ring/flow
        #                    + vertical(loom) + yaw stay live.
        #   case (a) ABORT   if at corner-exit |s_e_n| >= TC_SEN (off-center) OR ds_e_n > 0
        #                    growing (s_e_n diverging = kick precursor). Sets _abort_requested
        #                    for the app to ascend + re-attempt.
        # The |ds_e_n| gate replaces the ||h_xy|| flow guard (same v_res info, centroid-robust,
        # matching s_e_n coordinate). All triggers image-space -> scale/depth-free. Default-off.
        self._terminal_commit = os.environ.get("PLASMC_TERMINAL_COMMIT", "0") == "1"  # BAKED OFF 2026-07-03 (user): the open-loop hold (zero zeta_r) is WRONG for a MOVING target — it drops position regulation and only velocity-matches, so a curving/translating deck develops an un-corrected offset; it also fired the r1.0 curved fly-away (peak 9.2->5.0 m with it off). On the STATIONARY platform it rarely fires anyway (extent<400 at platform min-alt ~0.5 m) and commit-off is neutral-to-better. Was BAKED ON 2026-06-29 (stationary s_e_n->0 ramp). Set =1 to restore.
        self._tc_extent = float(os.environ.get("PLASMC_TC_EXTENT", "400"))   # corner-exit: marker px extent
        self._tc_sen    = float(os.environ.get("PLASMC_TC_SEN",    "0.3"))   # |s_e_n| centered cut
        self._tc_dsen   = float(os.environ.get("PLASMC_TC_DSEN",   "0.2"))   # |ds_e_n| settled cut
        self._tc_frames = int(os.environ.get("PLASMC_TC_FRAMES",   "3"))     # consecutive-frame confirm
        # Smooth handoff: ramp the zeta_r contribution 1->0 over TC_RAMP_S after commit (a HARD
        # zero steps sigma_xy by chi_r*zeta_r ~0.27 at TC_SEN=0.3 -> a_u jolt). 0 = hard zero.
        self._tc_ramp_s = float(os.environ.get("PLASMC_TC_RAMP_S", "0.3"))
        # Option-A post-commit integral: replace the (faded) chi_r*zeta_r position barrier with a
        # PI-on-flow term Omega_xy*int(zeta_h_xy), mirroring the bounded z-axis (sigma_z = zeta_h_z +
        # Omega_z*int(zeta_h_z)). The integral is RESET to 0 at commit so its reference is the
        # CENTERED commit position (holds it; captures only post-commit drift, no approach windup).
        # Pair with loose P2INF_xy (low barrier gain) = the z-mirror (PI-on-flow, no terminal kick).
        self._tc_integral = os.environ.get("PLASMC_TC_INTEGRAL", "0") == "1"
        self._tc_omega_xy = float(os.environ.get("PLASMC_TC_OMEGA_XY", "0.05"))  # PI gain (cf Omega_z)
        self._tc_izeta_reset_done = False   # one-shot: izeta_xy reset at the commit step
        # Singhal-containment sign handling: the legacy form holds |last-good| * sign(CURRENT breach),
        # so when the terminal limit cycle flips sign the contained value flips with it -> the
        # containment FEEDS the sign-flipping. With this on, the LATERAL axes hold the full last-good
        # value (sign + magnitude) -> the containment freezes through the breach instead of toggling.
        # z keeps trust-sign (loom glitches preserve sign; that protection is load-bearing). Default-off.
        self._contain_hold_full = os.environ.get("PLASMC_CONTAIN_HOLD_FULL", "0") == "1"
        # IDEA 1 — SOFT funnel-breach handling (2026-06-30, user). At a breach the hard clamp pins the
        # feature at the funnel edge (S_margin) where G/g_r are LARGEST -> sigma blows up AND control is
        # starved (a_u=-G^-1*a_v->0); the containment additionally HOLDS h_e (freezes zeta_h, the velocity
        # feedback) + freezes kappa. Instead: pull the breaching feature to FRAC*(last in-funnel value) ->
        # OFF the singular edge -> G/g_r bounded -> zeta_h/zeta_r stay LIVE (not saturated/frozen) AND
        # kappa_eq stays bounded (so no kappa freeze AND no runaway — resolves the containment-off dilemma).
        # Applied to BOTH the flow barrier (h_e, in the containment block below) and the position barrier
        # (s_e_n/zeta_r). Env-gated, default-off.
        self._soft_breach      = os.environ.get("PLASMC_SOFT_BREACH", "0") == "1"
        self._soft_breach_frac = float(os.environ.get("PLASMC_SOFT_BREACH_FRAC", "0.30"))
        if self._soft_breach:
            print(f"[controller] PLASMC_SOFT_BREACH=1: soft funnel-breach handling frac={self._soft_breach_frac} "
                  "(pull breaching h_e & s_e_n to frac*last-in-funnel; kappa keeps adapting, no freeze)")
        # LOOM-INVERSION TOUCHDOWN DETECTOR (depth-free, soft-touchdown). Throughout the descent the
        # loom h_z = v_z/Z tracks h_rd<0 (contraction); it flips POSITIVE only when the drone reverses
        # vertically at first ground contact (rebound). The accel-spike detector (|a|>50) misses a SOFT
        # touchdown (gentle contact ~20 m/s², below threshold + ambiguous with descent transients), and
        # PX4 ON_GROUND needs the drone settled (the bounce defeats it). Verified: the FIRST sustained
        # h_z>0 (neg->pos) marks the first contact (Z~0.11-0.14); the pre-contact descent has zero
        # inversions. Fire -> LANDED so the app disarms BEFORE the control pumps the bounce. Scale-free,
        # moving-target-OK, closed-loop until contact (NOT an open-loop commit). Default-off.
        self._touchdown_loom = os.environ.get("PLASMC_TOUCHDOWN_LOOM", "1") == "1"  # BAKED ON 2026-06-29 (loom-inversion touchdown detect)
        self._td_frames = int(os.environ.get("PLASMC_TD_FRAMES", "3"))    # persistence (reject noise spikes)
        self._td_sen    = float(os.environ.get("PLASMC_TD_SEN", "0.6"))   # near-centered gate (first contacts 0.34-0.53)
        self._td_arm_loom = float(os.environ.get("PLASMC_TD_ARM_LOOM", "-0.1"))  # arm once a descent is established
        self._td_streak = 0
        self._td_armed  = False
        self._touchdown = False
        if self._touchdown_loom:
            print(f"[controller] PLASMC_TOUCHDOWN_LOOM=1: loom-inversion touchdown detect "
                  f"(h_z>0 for {self._td_frames} frames + |s_e_n|<{self._td_sen}, armed after h_z<{self._td_arm_loom})")
        # SAVGOL FORWARD-PREDICTOR (lag compensation, idea 2). The 38 ms loop delay makes the lateral
        # velocity loop under-damped near the deck -> the limit cycle. A FORWARD predictor (fit a
        # degree-D poly to the last WIN image samples, evaluate at t+LEAD) un-lags the control: it
        # estimates where h/s ACTUALLY are now (vs the lagged measurement) -> restores phase margin.
        # SCALE-FREE (operates on the image observables h, s directly; no Z). Tests whether
        # lag-compensation drops the relay% WITHOUT loosening the funnel. Default-off.
        # GATED: the buffer (self._h, self._s) fills the WHOLE flight (always warm), but the savgol
        # prediction is only APPLIED on frames where the rate |dh|/|ds| exceeds a physical limit (the
        # 1/Z+lag-corrupted "wrongly estimated" spike, idea 2's gate). By the terminal the window is
        # FULL (filled during the stable approach) -> no start-up transient. Lateral-only (leave the
        # loom/descent untouched). On a flagged frame, replace the spiked value with the savgol
        # extrapolation from the PRE-spike window (excludes the corrupted current sample).
        self._savgol_predict = os.environ.get("PLASMC_SAVGOL_PREDICT", "1") == "1"  # BAKED ON 2026-06-29 (model-predict lag comp)
        # PREDICTOR MODE: "model" = kinematic consistency (ṡ=h_lat−s·h_z; flow↔bearing reconstruct each
        # other from last-good values, no window — handles the 1/Z regime change savgol can't);
        # "savgol" = legacy polynomial extrapolation (dead-end, kept for A/B). Default model.
        self._predict_mode = os.environ.get("PLASMC_PREDICT_MODE", "model").lower()
        self._predict_lead = float(os.environ.get("PLASMC_PREDICT_LEAD_S", "0.04"))  # forward horizon (s; 0=just correct)
        self._predict_win  = int(os.environ.get("PLASMC_PREDICT_WIN", "15"))         # fit window (warm at terminal)
        self._predict_deg  = int(os.environ.get("PLASMC_PREDICT_DEG", "2"))          # poly degree (2=curvature)
        # BUG FIX (2026-08-04, see project_pi_dt_visibility_independence_2026_08_04
        # memory): max last-good-frame gap _predictModel_s will extrapolate over
        # before falling back to the raw measurement instead -- see that method's
        # own comment for the full writeup (dt bypasses the self._dt fix, and
        # MULTIPLIES the extrapolation term, so a large gap directly inflates it).
        self._predict_max_gap_s = float(os.environ.get("PLASMC_PREDICT_MAX_GAP_S", "0.3"))
        # PER-AXIS rate-gate limits — the loom (z) has a different scale than the lateral (x,y), so
        # its limit is separate. Derived from the terminal-kick data (approach p90 << limit << terminal p90).
        _dh_xy = float(os.environ.get("PLASMC_DH_LIMIT_XY", "10.0"))
        _dh_z  = float(os.environ.get("PLASMC_DH_LIMIT_Z",  "12.0"))
        self._dh_limit = np.array([_dh_xy, _dh_xy, _dh_z])             # per-axis |dh_k| gate
        self._ds_limit = np.array([float(os.environ.get("PLASMC_DS_LIMIT", "5.0"))] * 2)  # |ds_n_k| gate (x,y)
        # IMMUTABLE raw buffers (EKF/KF measurement stream) — the gate reads + the savgol fits from
        # THESE, never from the corrected self._h/self._s. Predictions write ONLY to self._h/self._s.
        # Breaks the self-feedback loop (prediction never re-enters the fit) and makes the gate read
        # the TRUE raw rate (terminal-selective, not the corrupted 75%-at-altitude). _good = per-frame
        # mask (False when any axis spiked) so the fit skips genuine spikes too.
        self._h_raw = []; self._s_raw = []
        self._h_good = []; self._s_good = []
        if self._savgol_predict:
            print(f"[controller] PLASMC_SAVGOL_PREDICT=1: PER-AXIS rate-gated predictor MODE={self._predict_mode} "
                  f"(lead={self._predict_lead}s; gate |dh|>{self._dh_limit} |ds_n|>{self._ds_limit}; "
                  f"raw-buffer, no feedback)")
        self._tc_commit_t = None      # sim time at case-(b) commit (taper origin)
        self._tc_count  = 0           # consecutive frames the case-(b) condition has held
        self._tc_seen_exit = False    # corner-exit event has fired at least once
        self._abort_requested = False # case (a) — exposed to the app for re-ascend
        self._abort_reason = ""
        if self._terminal_commit:
            print(f"[controller] PLASMC_TERMINAL_COMMIT=1: corner-exit commit "
                  f"(extent>={self._tc_extent}, |s_e_n|<{self._tc_sen}, h_z<0, "
                  f"|ds_e_n|<{self._tc_dsen}, {self._tc_frames}-frame) -> zero zeta_r; "
                  f"off-center/diverging -> abort")
        # LEVER 2 (flow ceiling + smoothing): blend the accurate DETECTED-centroid rate
        # d(s[:2])/dt into the lateral flow h[:2]. The LK flow saturates ~1 rad/s AND
        # carries lstsq garbage spikes; the centroid rate has no LK ceiling and no lstsq
        # garbage (clean detected position -> clean derivative) -> raises the ceiling AND
        # smooths in one move (both s and h are V-frame, so d(s)/dt IS the translational
        # flow). FLOW_CENTROID_RATE in [0,1]: 0 = pure LK (default OFF), 1 = pure centroid.
        self._FLOW_CENTROID_RATE = float(os.environ.get("FLOW_CENTROID_RATE", "0.0"))
        # MANUSCRIPT c_h (§II correction) — BAKED default-ON 2026-06-25. Clean form
        # c = -psi_dot_b*(e3 x h) - (h.e3)*h - dh_d : drops the SUPERSEDED camera-frame
        # static-target cross-products (omega_dot x s, omega x(omega x s), 2 omega x h) that the old
        # form transplanted. The manuscript main text carries this clean form; the old cross-product
        # form (PLASMC_CH_CLEAN=0, retained for parity) was the mis-derived one. Baked to align the
        # code with the manuscript formula. NOTE: this is a FORMULATION correction; it does NOT fix
        # the terminal deck fly-away (a separate Task-2/optic-flow limit-cycle: h_e=v_rel/Z diverges
        # at the deck when v_rel doesn't reach 0 -- see feedback_terminal_launch_flow_loop + the
        # SP/limit-cycle analysis). psi_dot_b sign still SITL-unverified (PLASMC_CH_PSIDOT_SIGN knob).
        self._CH_CLEAN = os.environ.get("PLASMC_CH_CLEAN", "1") == "1"
        self._ch_psidot_sign = float(os.environ.get("PLASMC_CH_PSIDOT_SIGN", "1.0"))  # flip transport-term yaw rate (clean c-term); -1 retests the regressed sign

        # Low-pass filter on inertial accel (MATLAB: tau_ia = 0.08 s)
        self._tau_ia = 0.08
        # LPF on yaw rate command. The s[3] marker-orientation feature wraps
        # at ±π/2 due to 180° symmetry of the moment estimator; single-pixel
        # noise near the boundary flips e_a by π, producing ~100°/s u_a step
        # commands that PX4's motor mixer translates into vertical-thrust
        # ripple (T ∝ ω²). LPF removes the high-frequency switching while
        # preserving the slow yaw-drift correction.
        self._tau_ua = float(os.environ.get("PLASMC_TAU_UA", "0.1"))

        # Anti-windup clamps (MATLAB izeta_2 clamped to ±5; others heuristic)
        self._iV_s_e_n_clamp = 5.0
        # Per-component sliding-surface integral anti-windup cap. Matches
        # Supplement S2-D item 2 (corrected): |∫ζ_{2k}dτ| ≤ 5, chosen so the
        # integral contribution X·∫ζ to σ stays ≤ λ_max(X)·5 ≤ 0.25, within
        # the boundary-layer thickness ε_k = 1 used in the simulations.
        self._izeta_clamp = 5.0
        # _ie_a_clamp REMOVED 2026-06-08 — the fixed yaw-integral clamp was a band-aid
        # that masked integral windup during large initial-yaw slews (and drove the
        # post-slew overshoot). Replaced by CONDITIONAL INTEGRATION in _yawCtrl
        # (freeze ie_a while the heading rate is saturated) — proper anti-windup.

        # ════ SO(3) attitude-error proportional gain  [manuscript: k_R] ════
        # DIRECT per-axis values in (rad/s)/rad — rate-mode unit, not the
        # manuscript's N·m/rad (the damping k_Ω lives inside PX4's rate loop).
        # Default 2.0: higher overdrives the LK tracking window (>1.7 rad/s
        # body rate → >15 px/frame corner motion → OPTIC FLOW UNAVAILABLE).
        # Works together with the PLASMC_W_U_MAX=1.0 rad/s command clamp.
        self._K_R = np.diag([float(os.environ.get("PLASMC_KR_ROLL",  "2.5")),
                             float(os.environ.get("PLASMC_KR_PITCH", "2.5")),
                             float(os.environ.get("PLASMC_KR_YAW",   "0.5"))])   # 2/2/2->1.5/1.5/0.5 (2026-06-12), rp 1.5->2.5 (2026-06-24): LATERAL mid-descent limit-cycle fix (per-axis GT-FB campaign NC125-126). The lateral outer loop is under-damped (not unstable); the binding limit was the inner-loop attitude lag (eR_pitch -22deg vs cmd 33deg). roll/pitch rate loop is FAST (~40ms) so K_R rp^ cuts the lag (-22->+-5deg) -> restores effective damping -> mid-descent lateral cycle eliminated (s_e_n 0.05-0.14, ncross4, alt 4->1.5m). YAW stays 0.5: K_R_YAW^ RULED OUT (worse) - yaw rate loop is SLOW (287ms) so stiffening over-drives the lag; yaw under-damping is the outer windup + rate lag (fixed by Omega_a->0.1), NOT attitude tracking. Validated GT-FB; raises body rate -> affects image-mode LK window (sharp optimum, GT-FB-validated only).

        # Virtual-compass heading state. Lazy-init on first _attCtrl call
        # from the current body yaw (manuscript: psi_d = yaw_init, line 127
        # of visualControl_IBVS_adaptive.m).
        self._psi_d = None

        # Reference depth-rate (MATLAB h_rd was -0.42; user has chosen -0.30 historically via REF_RAD_OPT_FLOW)
        # We keep ref_rad_opt_flow from the constructor; do not override.

        # Log segments archived across mid-flight re-initializations: a full visibility
        # loss re-inits the controller, which WIPES the live log lists — losing exactly
        # the initial-approach data needed to diagnose the failure that caused the loss.
        # _archive in run() snapshots the lists before each re-init; getLogData merges.
        self._log_segments = []

        # BUG FIX (2026-08-04, see project_pi_dt_visibility_independence_2026_08_04
        # memory): a pure wall-clock heartbeat, refreshed EVERY run() loop
        # iteration regardless of marker visibility (see run()/_updateTime()) --
        # deliberately initialized here in true __init__, NOT inside
        # _initialize_controller() (which re-runs on every marker-loss reinit),
        # so this keeps ticking continuously across reinits too.
        self._last_loop_t = None
        self._last_loop_dt = None

        self._initialize_controller()
        self.start()

    def __del__(self):
        print("Controller thread is deleted...")

    def close(self):
        self._STAY_OPEN = False

    @property
    def FEATURE_IS_STALE(self):
        """Forwards img_data's stale-feature flag (see Intervention 2,
        2026-05-22).  Lets landing_test refuse to act on extrapolated
        feature data after STALE_THRESH consecutive detection misses."""
        return bool(getattr(self._img_node, "FEATURE_IS_STALE", False))

    @property
    def CBF_CORNERS_STALE(self):
        """True once cbf_corners has been None (neither the PlanarFeatureMap
        small-slot nor raw _feature_pts source available) for
        CBF_CORNERS_STALE_FRAMES consecutive control-loop calls -- see the
        staleness-tracking comment at the cbf_corners selection site and
        PX4_Gazebo/docs/HANDOFF_cbf_lockout_planarmap_2026-07-30.md. Frame
        count default (30) matches roughly the same ~1s window
        MARKER_LOSS_GRACE's default uses at this loop's typical rate;
        override via CBF_CORNERS_STALE_FRAMES if the live rate differs
        meaningfully."""
        _frames = int(os.environ.get("CBF_CORNERS_STALE_FRAMES", "30"))
        return getattr(self, "_cbf_corners_none_streak", 0) >= _frames

    @property
    def CBF_CORNERS_STALE_ABORT(self):
        """Separate, much longer-fused staleness check for MISSION-ABORT decisions only
        (hardware_landing.py's feature_fresh -> marker-loss-grace -> RTL handoff), added
        2026-07-31 after CBF_CORNERS_STALE's fast ~30-frame/1s threshold was found to
        false-trip on ordinary ArUco coast bursts: the 2026-07-27 root-cause investigation
        (project_pi_coast_root_cause) established that NORMAL, non-failure coast bursts on
        this hardware commonly run 2-327 frames even while otherwise tracking fine (narrow
        HFOV, small marker). Reusing the fast threshold for an irreversible mission-abort
        meant nearly every flight in the 2026-07-31 test session (15 of 17 attempts) aborted
        to RTL within seconds, never reaching a sustained closed-loop descent. The fast
        CBF_CORNERS_STALE stays as-is for the kappa-freeze use below (low-risk, just pauses
        adaptation) -- only the abort path gets this longer fuse. Default 350 frames sits
        just past the observed max normal coast burst (327); override via
        CBF_CORNERS_STALE_ABORT_FRAMES if the live rate or marker/FoV geometry changes."""
        _frames = int(os.environ.get("CBF_CORNERS_STALE_ABORT_FRAMES", "350"))
        return getattr(self, "_cbf_corners_none_streak", 0) >= _frames

    @property
    def CBF_OVERFLOW(self):
        """True iff the CBF's own per-corner FoV-margin classification found the current
        CBF corner source (small-marker-preferred, see cone-angle computation) breaching
        the margin on OPPOSITE sides (spanning -- still over target, benign). Signals
        handover-readiness on the BIG marker; a future consumer could combine this with
        small-slot map confidence to decide whether to actually switch primary. The CBF
        only SIGNALS here; it does not force any switch itself."""
        return bool(self._cbf_overflow)

    @property
    def CBF_DRIFT_OFF(self):
        """True iff the CBF's own per-corner FoV-margin classification found a ONE-SIDED
        breach (target visibility genuinely failing) -- the CBF's own job to prevent, not
        just observe."""
        return bool(self._cbf_drift_off)

    @property
    def MARKER_EXTENT_PX(self):
        """Marker SPAN (px) — corner-to-corner size of the latest detection.
        SCALE-FREE proximity indicator: a large span means the marker is BIG
        in the image, i.e. touchdown is geometrically imminent — no depth or
        altitude is used. Used by landing_test's stale-streak commitment
        (env LANDING_STALE_COMMIT_EXTENT). Returns 0.0 when no corners exist.

        NOTE (batch-13 bug fix): the first version measured max distance from
        the image CENTER, which is also large when a small marker drifts to
        the image EDGE (lateral error at altitude) — that caused 3 false-
        positive commitments (1.5-2.5 m soft landings). Span distinguishes
        the two cases: big-marker-anywhere vs small-marker-at-edge."""
        try:
            fp_list = self._img_node._feature_pts
            if len(fp_list) > 0:
                raw_corners = np.asarray(fp_list[-1][1], dtype=float)
                u_span = raw_corners[:, 0].max() - raw_corners[:, 0].min()
                v_span = raw_corners[:, 1].max() - raw_corners[:, 1].min()
                return float(max(u_span, v_span))
        except (IndexError, AttributeError, ValueError, TypeError):
            pass
        return 0.0

    def _terminalCommitStep(self, s_e_n, ds_e_n):
        """Terminal-commit discriminator (TERMINAL_KICK_COMMIT_DESIGN.md). Called each step in
        combined mode with the live normalized error s_e_n (=r_bar_e) and its FILTERED rate
        ds_e_n (=dr_bar_e = s_dot_meas/p_10, CV-KF). Latches case (b) commit or flags case (a)
        abort. Scale/depth-free: extent(px), s_e_n, h_z, ds_e_n are all image-space."""
        if self._committed:
            return                                          # one-way latch — never re-evaluate
        # Corner-exit event: MEDIAN marker extent over the window crosses the FoV-fill threshold
        # (median filters the switching-noisy raw extent that spuriously spikes — see _commit_win).
        self._ext_win.append(float(self.MARKER_EXTENT_PX))
        if not (len(self._ext_win) >= self._commit_win
                and float(np.median(self._ext_win)) > self._tc_extent):
            self._tc_count = 0
            return
        self._tc_seen_exit = True
        sen  = float(np.linalg.norm(s_e_n))
        dsen = float(np.linalg.norm(ds_e_n))
        h_z  = float(self._h[-1][2]) if len(self._h) > 0 else 0.0
        centered = sen  < self._tc_sen
        looming  = h_z  < 0.0
        settled  = dsen < self._tc_dsen
        # case (a) ABORT: off-center at corner-exit, OR s_e_n actively DIVERGING (rate points
        # outward, ds_e_n . s_e_n > 0, at non-trivial magnitude = the breach/kick precursor).
        diverging = float(np.dot(ds_e_n, s_e_n)) > 0.0 and dsen >= self._tc_dsen
        if (not centered) or diverging:
            self._tc_count = 0
            if not self._abort_requested:
                self._abort_requested = True
                self._abort_reason = "off-center" if not centered else "diverging"
                print(f"[controller] TERMINAL-COMMIT case(a) ABORT ({self._abort_reason}): "
                      f"|s_e_n|={sen:.3f} |ds_e_n|={dsen:.3f} extent={np.median(self._ext_win):.0f}")
            return
        # case (b) COMMIT candidate: centered + looming + settled. Confirm over TC_FRAMES.
        if centered and looming and settled:
            self._tc_count += 1
            if self._tc_count >= self._tc_frames:
                self._committed = True
                self._tc_commit_t = self._t[-1]
                self._s_e_n_hold = np.asarray(s_e_n).copy()
                print(f"[controller] TERMINAL-COMMIT case(b) COMMIT: |s_e_n|={sen:.3f} "
                      f"|ds_e_n|={dsen:.3f} h_z={h_z:.2f} extent={np.median(self._ext_win):.0f} "
                      f"-> zeta_r zeroed (kick removed)")
        else:
            self._tc_count = 0   # centered but still converging (not settled): wait, don't latch

    def _ringCommitStep(self, s_e_n, ds_e_n):
        """Terminal RING-commit gate (project_terminal_velocity_handover_design). Latches
        _ring_committed (one-way) on: img HANDOVER_LATCHED (big->small, depth-free entered-terminal)
        AND OVER_TARGET (2026-07-05 user: nadir inside the small marker's V-frame quad -> we are over the
        target; RELAXED from the s_e_n bearing gate, which was ambiguous at altitude) AND settled
        (|ds_e_n|<TC_DSEN, velocity-matched -> moving-target-valid). On commit the _updateOptFlow override
        swaps h_xy->0 + h_z->ring loom; zeta_r (marker-s position) stays LIVE."""
        if self._ring_committed:
            return
        if not getattr(self._img_node, 'HANDOVER_LATCHED', False):   # gate 1: entered terminal
            self._tc_count = 0
            return
        over_target = bool(getattr(self._img_node, 'OVER_TARGET', False))    # gate 2: nadir inside small marker (replaces s_e_n)
        settled     = float(np.linalg.norm(ds_e_n)) < self._tc_dsen          # gate 3: velocity-matched
        if over_target and settled:
            self._tc_count += 1
            if self._tc_count >= self._tc_frames:
                self._ring_committed = True
                print(f"[controller] RING-COMMIT: handover+over_target+settled -> h_xy=0, h_z=ring loom "
                      f"(|s_e_n|={float(np.linalg.norm(s_e_n)):.3f} |ds_e_n|={float(np.linalg.norm(ds_e_n)):.3f})")
        else:
            self._tc_count = 0

    @property
    def ABORT_REQUESTED(self):
        """Case-(a) signal for the app: corner-exit reached off-center / diverging -> re-ascend."""
        return self._abort_requested

    def _predictForward(self, raw, good, lead):
        """Savgol/poly FORWARD predictor reading ONLY the immutable `raw` buffer and GOOD (non-spiked)
        frames -> no self-feedback, no spike contamination. Fit a degree-`_predict_deg` poly to the
        last `_predict_win` good raw samples (vs control time, all strictly before NOW), then EVALUATE
        at NOW+lead -> de-glitched, lag-compensated estimate. Scale-free. Falls back to the raw last
        value when there aren't enough good frames or stamps are non-monotone."""
        win, deg = self._predict_win, self._predict_deg
        lo = max(0, len(raw) - 1 - 4 * win)                       # bound the search to recent frames
        idxs = [i for i in range(lo, len(raw) - 1) if i < len(good) and good[i]]   # good frames before NOW
        if len(idxs) < win or len(self._t) < len(raw):
            return raw[-1]
        idxs = idxs[-win:]
        tt = np.asarray([self._t[i] for i in idxs], float) - float(self._t[-1])    # relative to NOW (now=0)
        if not np.all(np.diff(tt) > 1e-6):
            return raw[-1]
        Y = np.asarray([raw[i] for i in idxs], float)
        out = np.empty(Y.shape[1])
        for k in range(Y.shape[1]):
            out[k] = np.polyval(np.polyfit(tt, Y[:, k], deg), lead)   # lead from NOW
        return out

    def _lastGood(self, good, n=1):
        """Up to `n` most-recent indices i with good[i]==True (ascending), searched over recent frames."""
        out = []
        for i in range(len(good) - 1, max(-1, len(good) - 1 - 6 * self._predict_win), -1):
            if good[i]:
                out.append(i)
                if len(out) == n:
                    break
        return out[::-1]

    def _predictModel_s(self, k, lead):
        """MODEL-based bearing estimate for spiked lateral axis k, from the kinematics ṡ=h_lat−s·h_z:
        s_k = s_prev_good + (h_lat_k − s_prev_good·h_z)·(Δt+lead). Anchored to last-good bearing +
        last-good flow/loom (live values carry the 1/Z regime; no window, no extrapolation overshoot)."""
        gi = self._lastGood(self._s_good, 1); hi = self._lastGood(self._h_good, 1)
        if not gi or not hi:
            return self._s_raw[-1][k]
        ip, ih = gi[-1], hi[-1]
        s_prev = self._s_raw[ip][k]; h_lat = self._h_raw[ih][k]; h_z = self._h_raw[ih][2]
        raw_gap = float(self._t[-1]) - float(self._t[ip])
        # BUG FIX (2026-08-04, see project_pi_dt_visibility_independence_2026_08_04
        # memory): dt here is computed directly from self._t (last-good frame ->
        # now), bypassing the self._dt fix entirely -- self._t/_s_raw/_s_good only
        # grow on visible ticks, so ip can point to a pre-gap entry when the first
        # fresh measurement after a marker-loss gap looks "spiked" relative to
        # stale history. Unlike dw/_predictModel_h's sdot (dt in the denominator --
        # a large gap only damps those, doesn't blow them up), dt MULTIPLIES the
        # extrapolation term here, so a large gap directly inflates the output.
        # Same fallback philosophy as _predictForward ("falls back to the raw last
        # value when there aren't enough good frames"): beyond a sane gap, don't
        # trust a linear extrapolation that far out -- fall back to the raw
        # measurement instead of extrapolating with a huge dt.
        if raw_gap > self._predict_max_gap_s:
            return self._s_raw[-1][k]
        dt = max(raw_gap, 1e-3) + lead
        return s_prev + (h_lat - s_prev * h_z) * dt

    def _predictModel_h(self, k, lead):
        """MODEL-based flow estimate for spiked lateral axis k, inverting the kinematics:
        h_lat_k = ṡ_k + s_k·h_z, with ṡ_k the last-good bearing rate. Reconstructs the (velocity-like)
        flow from the smoother (position-like) bearing + measured loom."""
        gi = self._lastGood(self._s_good, 2); hi = self._lastGood(self._h_good, 1)
        if len(gi) < 2 or not hi:
            return self._h_raw[-1][k]
        i0, i1, ih = gi[0], gi[1], hi[-1]
        sdot = (self._s_raw[i1][k] - self._s_raw[i0][k]) / max(float(self._t[i1]) - float(self._t[i0]), 1e-3)
        return sdot + self._s_raw[i1][k] * self._h_raw[ih][2]

    def _touchdownDetect(self, s_e_n):
        """Loom-inversion soft-touchdown detector. Arms once a descent is established (h_z < arm),
        then latches LANDED when the loom holds POSITIVE for _td_frames frames (= the vertical
        reversal at first contact) while near-centered. Depth-free (loom only), one-way latch."""
        # DISABLED 2026-07-29 (user decision): armed on a single spiked h_z frame
        # (-0.57, held constant 10 ticks -- a stale/corrupted frame, not real loom)
        # while the drone was still climbing at 3-4m, then latched LANDED off
        # ordinary positive h_z noise a few frames later -- false touchdown,
        # mid-air. No persistence/spike-rejection gate on the arm condition (see
        # controller.py:851 history). Re-enable once the arm condition is gated
        # on _h_good and/or requires multi-frame persistence, not a single frame.
        return
        if not self._touchdown_loom or self._touchdown:
            return
        h_z = float(self._h[-1][2])
        if not self._td_armed:
            if h_z < self._td_arm_loom:      # a real descent has been seen -> arm
                self._td_armed = True
            return
        self._td_streak = self._td_streak + 1 if h_z > 0.0 else 0
        if self._td_streak >= self._td_frames and float(np.max(np.abs(s_e_n))) < self._td_sen:
            self._touchdown = True
            print(f"[controller] TOUCHDOWN-DETECT: loom inverted (h_z>0 x{self._td_frames}) "
                  f"|s_e_n|={float(np.max(np.abs(s_e_n))):.2f} -> LANDED (disarm before bounce)")

    @property
    def TOUCHDOWN_DETECTED(self):
        """One-way latch: first persistent loom inversion near-centered -> first ground contact.
        The app reads this to disarm at touchdown (the accel-spike detector misses soft contacts)."""
        return self._touchdown

    @property
    def LATERAL_ERR_N(self):
        """|s_e_n| — norm of the latest normalized lateral image error (scale-free,
        image-only). Used by landing_test's proximity commitment to require the drone
        be CENTERED (not just close) before the open-loop touchdown handoff. Returns
        inf when no error has been computed yet (blocks the commit)."""
        try:
            if len(self._s_e_n) > 0:
                return float(np.linalg.norm(self._s_e_n[-1]))
        except (IndexError, AttributeError, ValueError, TypeError):
            pass
        return float('inf')

    @property
    def LOOM_Z(self):
        """h_z — the z-component (loom / flow divergence) of the latest calibrated
        optic flow (= vz/Z, rad/s). SCALE-FREE and image-only (no depth/altitude).
        In GT-feedback mode this is the GT-derived loom, so it is perception-free.
        Used by landing_test's loom-accumulation commitment: a clean h_rd descent
        holds h_z ≈ h_rd (constant), so the INSTANTANEOUS loom is not a proximity
        signal — the harness integrates it (∫h_z dt = ln(Z/Z_start)) for a scale-
        free proximity. Returns 0.0 (no accumulation) before any flow is computed."""
        try:
            if len(self._h) > 0:
                return float(self._h[-1][2])
        except (IndexError, AttributeError, ValueError, TypeError):
            pass
        return 0.0

    def _initialize_controller(self):
        # Attitude state
        self._quat = []
        self._w_i = []     # camera-frame angular velocity from img_data
        self._w = []       # body angular velocity (full 3-axis, FRD->body-NED)
        self._dw = []
        self._dw_deque = deque([np.zeros(N_DIM)] * 4)
        self._w_i_last_t = None       # time of last w_i change → image-rate dw magnitude (2026-06-10)

        # Optical flow / middle-loop state
        self._h = []
        self._h_d = []
        self._h_e = []
        self._dh_d = []
        self._dh_d_deque = deque([np.zeros(N_DIM)] * 4)
        self._s_rate_deque = deque([np.zeros(2)] * 4)   # LEVER 2: smoothed centroid-rate

        # Image features
        self._s = []
        self._s_e = []        # raw error (s - s_d)
        self._s_e_n = []      # normalized error (s_e[:2] / p_10)
        self._is_e_n = []     # integral of s_e_n
        self._ds_e_n_deque = deque([np.zeros(2)] * 4)
        self._ds_d = []       # desired feature-derivative output of outer PID (LPF-smoothed)
        self._ds_d_lpf = np.zeros(N_DIM)   # LPF state for ds_d smoothing
        self._ds_d_lpf_init = False

        # Middle-loop barrier (optical flow)
        self._p = []
        self._dp = []
        self._S = []
        self._zeta = []
        self._izeta = []
        self._G = []
        # Outer-loop position barrier (s_e_n funnel) — mirrors the middle-loop barrier
        self._p_s = []
        self._dp_s = []
        self._S_s = []
        self._zeta_s = []
        self._izeta_s = []
        self._G_s = []
        self._dzeta_s_deque = deque([np.zeros(2)] * 4)
        # Combined-barrier position barrier (zeta_r on r_bar_e = s_e_n) + measured centroid rate
        self._p_r = []
        self._dp_r = []
        self._zeta_r = []
        self._dzeta_r = []
        self._s_dot_meas = []          # measured centroid rate (h_d feedforward, combined mode)
        self._s_dot_deque = deque([np.zeros(2)] * 4)
        # Un-degenerate zeta_h: use the funnel-tracking reference rate S_r*dp_r (= the back-map's
        # bounded S_s*dp_s term, NO G_s^-1 starvation) as the h_d x/y rate FF instead of measured
        # s_dot. Then h_e = h - h_d is a GENUINE velocity error (no s_dot self-cancellation) ->
        # zeta_h provides velocity damping alive at the converged center -> arrests the terminal
        # drift before s_e_n breaches the funnel. Convergence stays with zeta_r-in-sigma. Default-off.
        # UN-BAKED 2026-06-27: regressed the stationary gate (8/25 -> 1/25, gate-ON) — the loom-commit
        # already handles the stationary terminal, so the funnel-ref's live recovery fights it. KEPT
        # env-gated as the MOVING-TARGET (rover) candidate (there the commit is impossible, must track
        # live to touchdown). Default-off restores the s_dot_meas 8/25 stationary config.
        self._hd_funnel_ref = os.environ.get("PLASMC_HD_FUNNEL_REF", "1") == "1"  # BAKED ON 2026-06-29 (funnel-ref h_d; un-degenerate zeta_h)
        if self._hd_funnel_ref:
            print("[controller] PLASMC_HD_FUNNEL_REF=1: h_d x/y rate = S_r*dp_r (funnel ref); zeta_h un-degenerated (moving-target candidate)")
        # Back-map V_ds_e in the combined h_d: ds_d = p_10*(S_r*dp_r - k_r*zeta_r/g_r), i.e. the back-map's
        # inv(g_r)*dzeta_rd + S_r*dp_r with proportional dzeta_rd = -k_r*zeta_r (prescribe zeta_r_dot=-k_r*zeta_r
        # -> s_e_n converges at k_lat = |h_rd|+k_r with h_rd fixed; active convergence/recovery). The -k_r*zeta_r/g_r
        # carries the G_s^-1 (starves at edge) and rides into dh_d (kept, not dropped). k_r=0 -> funnel-only bake.
        self._hd_kr = float(os.environ.get("PLASMC_HD_KR", "0.5"))  # BAKED 0.5 2026-06-29
        if self._hd_kr != 0.0:
            print(f"[controller] PLASMC_HD_KR={self._hd_kr}: h_d carries back-map convergence term -k_r*zeta_r/g_r")
        # PLASMC_DHD_SRC (2026-07-02): WHICH h_d list feeds dh_d (-> c3 = -dh_d) in combined+funnel-ref
        # mode. The 06-27 "differentiate the FULL h_d honestly" call was premised on the rate term being
        # SMOOTH — true for S_r*dp_r, NOT for the -k_r*zeta_r/g_r branch baked 06-29 (barrier-inflated,
        # s_ddot-class; carries ~half of c3's content at the rover-curve cycle fundamental ~1.7 rad/s).
        # k_r's FUNCTION (recovery demand in h_e, via h_d) is separate from its DERIVATIVE (noise in c3):
        #   full (default, 06-29 behavior): differentiate the full h_d (rate incl. the k_r branch)
        #   nokr: differentiate h_d_noS + p_10*S_r*dp_r — s_ddot-drop applied ONLY to the k_r branch
        #         (kappa absorbs it as d_h; the MATLAB-validated s_ddot-drop pattern, 2026-06-18)
        #   nos:  differentiate h_d_noS only (pre-06-29 s_ddot-drop of the entire rate term)
        self._dhd_src = os.environ.get("PLASMC_DHD_SRC", "full").strip().lower()
        if self._dhd_src not in ("full", "nokr", "nos"):
            raise ValueError(f"PLASMC_DHD_SRC={self._dhd_src!r} (use full|nokr|nos)")
        if self._dhd_src != "full":
            print(f"[controller] PLASMC_DHD_SRC={self._dhd_src}: dh_d drops the "
                  + ("k_r branch (keeps S_r*dp_r)" if self._dhd_src == "nokr" else "whole h_d rate term"))
        # PLASMC_HD_PASSIVE (2026-06-29, default-off): the CLEAN stacked-barrier design
        # (STACKED_BARRIER_BACKSTEPPING.md:25) — h_d = passive rotation/descent feedforward ONLY
        # (h_d_noS), NO desired-rate term at all (_hd_rate = 0). Then h_e = h - h_d_noS is the pure
        # velocity-like flow error -> zeta_h is the velocity coordinate -> sigma = zeta_h + chi_r*zeta_r
        # is a clean PD surface whose NON-VANISHING chi_r*zeta_r term is the SOLE s_e_n driver. Removes
        # the back-map -k_r*zeta_r/g_r whose authority collapses at the funnel edge (S_r=0.648). Overrides
        # HD_FUNNEL_REF/HD_KR on the lateral rate. NOT degenerate: h carries translational flow that the
        # passive FF does not (the old degeneracy was h_d = s_dot_meas, NOT passive FF). A/B GT-FB IC2-5.
        self._hd_passive = os.environ.get("PLASMC_HD_PASSIVE", "0") == "1"
        if self._hd_passive:
            print("[controller] PLASMC_HD_PASSIVE=1: h_d = passive FF only (no desired-rate); "
                  "chi_r*zeta_r is the sole s_e_n driver (clean stacked-barrier design)")
        self._h_d_noS = []             # h_d minus the s_dot term (transport+descent) -> dh_d drops s_ddot
        self._h_d_kfree = []           # h_d minus the -k_r*zeta_r/g_r branch only (dh_d 'nokr' source, 2026-07-02)
        self._hd_rate_log = []         # DIAG (2026-06-30): the _hd_rate term of h_d (funnel-ref vs s_dot) for zeta_h-degeneracy decomposition
        self._theta = []      # ||Theta||_F
        self._sigma = []
        self._kappa = [self._kappa_0.copy()]

        # Yaw SMC state
        self._e_a = []
        self._ie_a = []
        self._sigma_a = []
        self._kappa_a = [np.array(self._kappa_a_0)]
        self._u_a = []        # commanded yaw rate (rad/s)

        # Attitude reference / SO(3) diagnostics
        # euler_d stores (phi_d, theta_d, psi_d) for backward-compatible
        # plotting; the active rate command comes from e_R, not Euler PD.
        self._euler_d = []
        self._e_R_log = []
        self._a_v = []
        self._a_u = []
        self._I_a_raw = []    # pre-LPF, pre-clamp inertial accel command
        self._I_a = []        # post-LPF, post-clamp inertial accel command
        # PLASMC_AU_LEAD (2026-07-03, user-approved): first-order phase-lead
        # C(s) = (1+s/wz)/(1+s/wp), wz<wp, on the LATERAL inertial command — applied to
        # I_a[:2] right after I_a_raw (BEFORE the cone clamp + tau_ia LPF, so the HF gain
        # lands on the cone). WHY (cycle deep-dive 07-03, project_rover_turning_open): the
        # rover-curve limit cycle is pumped by an anti-position command tone delivered
        # through the -25..-55 deg actuation lag; damping needs command quadrature
        # chi > W*tau ~ 25-40 deg at W* = 1.3-1.7 rad/s, and no gain knob can supply it
        # (all large branches sit at chi ~ 0). Defaults wz=0.9, wp=3.5 -> +35.5 deg and
        # gain x1.72 at 1.4 rad/s, HF gain wp/wz = 3.9 (bounded by the cone clamp).
        # I_a_raw stays logged PRE-lead so cycle_stage_phase.py measures the lead inside
        # the actuation stage (expect ACT(Iar->ad) to move ~ +35 deg when enabled).
        self._au_lead = os.environ.get("PLASMC_AU_LEAD", "0") == "1"
        self._au_lead_wz = float(os.environ.get("PLASMC_AU_LEAD_WZ", "0.9"))
        self._au_lead_wp = float(os.environ.get("PLASMC_AU_LEAD_WP", "3.5"))
        # AU_LEAD_RATIO (2026-07-03): SCALE-FREE clamp on the lead's EXTRA term
        # (wp/wz-1)(I_a_raw-x), bounded RELATIVE to the raw command: |delta| <= ratio*|I_a_raw_xy|.
        # Dimensionless (no fixed metric threshold) — rides the signal's own scale. The lead helps
        # mid-descent (delta small vs command -> transparent); the terminal 1/Z spike makes the
        # HF-amplified delta dwarf the command -> detonation, which the fraction cap neuters WITHOUT
        # an altitude/metric gate. 0 -> unclamped. Default 1.0 (delta <= the command it corrects).
        # (Superseded PLASMC_AU_LEAD_MAX, a fixed-m/s^2 clamp = a scale violation; removed.)
        self._au_lead_ratio = float(os.environ.get("PLASMC_AU_LEAD_RATIO", "1.0"))
        self._au_lead_x = np.zeros(2)          # LPF_wp state (world lateral)
        if self._au_lead:
            if not 0.0 < self._au_lead_wz < self._au_lead_wp:
                raise ValueError("PLASMC_AU_LEAD needs 0 < AU_LEAD_WZ < AU_LEAD_WP")
            _ph14 = np.degrees(np.arctan(1.4 / self._au_lead_wz)
                               - np.arctan(1.4 / self._au_lead_wp))
            print(f"[controller] PLASMC_AU_LEAD=1: lateral lead (1+s/{self._au_lead_wz:g})/"
                  f"(1+s/{self._au_lead_wp:g}) on I_a xy "
                  f"(+{_ph14:.0f} deg @1.4 rad/s, HF x{self._au_lead_wp/self._au_lead_wz:.1f})")
        self._marker_extent = []   # MARKER_EXTENT_PX per step (proximity / terminal-hold trigger)
        # FoV-cone diagnostics
        self._rho_fov_log = []
        self._d_min_fov_log = []
        self._theta_cone_log = []
        self._theta_current_log = []
        self._cbf_state = {}       # persistent cbf2 state (former _lw_*); see cbf_visibility.cbf2_filter
        self._theta_safe = None    # cbf2 Phase-1 safe lean vector (Fix B: direct->rd3)
        # CBF SMALL-MARKER PREFERENCE + OVERFLOW/DRIFT-OFF (2026-07-25 port from
        # PX4_Gazebo/src/controller.py, 2026-07-17 user design): the CBF needs more tilt
        # headroom than the flow pipeline does -- flow (h_x/h_y) needs the BIG marker's
        # wider corner spread to avoid a rank-deficient lstsq (stays big-priority,
        # unaffected by this), but the CBF's own FoV margin gets eaten by the big
        # marker's own overflow near touchdown, giving it LESS headroom right when it
        # needs more. Read the SMALL slot from PlanarFeatureMap (independent of which
        # marker is "primary" for flow/s) once it's confidently mapped; fall back to
        # whatever's live otherwise.
        self._cbf_small_conf_min = float(os.environ.get("CBF_SMALL_SLOT_CONF_MIN", "0.5"))
        # HYSTERESIS (found via Gazebo's IC4 regression on first validation pass): a raw
        # per-frame threshold check has ZERO persistence -- every other confidence gate in
        # this codebase (rescue gate, STALE_THRESH, _rigid_fail_streak) uses immediate-
        # off/N-frame-persistence-on hysteresis specifically to prevent flicker near the
        # boundary. Without it, whenever get_slot_confidence hovers near
        # CBF_SMALL_SLOT_CONF_MIN, the CBF's corner source (and therefore
        # d_min_fov/theta_cone) can jump discontinuously frame-to-frame between the big
        # marker's real geometry and the small marker's mapped geometry -- confirmed live
        # in Gazebo: theta_cone collapsed near-zero for a continuous 5.6s stretch
        # immediately preceding a NEW target_lost=DRIFT_OFF that wasn't present before
        # this change.
        self._cbf_small_slot_on = False
        self._cbf_small_slot_streak = 0
        self._cbf_small_slot_on_frames = int(os.environ.get("CBF_SMALL_SLOT_ON_FRAMES", "5"))
        # BOUNDED coast-hold for cbf_corners (ported from PX4_Gazebo/src/controller.py,
        # 2026-07-31) -- see the coast-hold comment at the cbf_corners selection site.
        # Default 30 frames; 0 = OFF (snap to no-corners on every coast frame).
        self._cbf_coast_grace_frames = int(os.environ.get("PLASMC_CBF_COAST_GRACE", "30"))
        self._cbf_coast_last_good = None
        self._cbf_coast_ctr = 0
        self._cbf_overflow = False
        self._cbf_drift_off = False
        self._cbf_drift_axis = None
        self._w_u = []
        self._B_T = []
        self._u = []

        # Time
        self._t = []
        self._dt = []
        self._t0 = self._time.perf_counter()
        # Savgol-predictor raw/good buffers MUST reset in lockstep with _t/_dt --
        # _lastGood() returns indices into these buffers that _predictModel_s/_h
        # then use directly to index self._t. Carrying stale (longer) s_raw/h_raw
        # across a marker-loss -> reacquisition transition desyncs the lengths and
        # crashes with IndexError on the first predict call after reacquisition.
        self._h_raw = []; self._s_raw = []
        self._h_good = []; self._s_good = []

    def run(self):
        while self._img_node.is_alive() and self._STAY_OPEN:
            # BUG FIX (2026-08-04, see project_pi_dt_visibility_independence_2026_08_04
            # memory): refresh the loop-wall-clock heartbeat EVERY iteration, BEFORE
            # the visibility check below -- independent of whether the marker is
            # currently visible. self._updateTime() uses self._last_loop_dt (not a
            # gap between visible-marker ticks) so dt always reflects the true small
            # per-loop-iteration period, never a multi-frame/multi-second visibility
            # gap. self._last_loop_t/_dt persist across marker-loss reinits (see
            # __init__, not _initialize_controller()) -- they're pure wall-clock
            # bookkeeping, not control-law state.
            _now_loop = self._time.perf_counter()
            if self._last_loop_t is not None:
                self._last_loop_dt = _now_loop - self._last_loop_t
            self._last_loop_t = _now_loop

            # GT-FEEDBACK: keep the loop alive on GT even if perception loses the
            # marker (the GT target pose is always available) — isolates control
            # from the perception-visibility gate through terminal descent.
            if self._img_node.FEATURE_IS_VISIBLE or self._gt_feedback is not None:

                if self._CONTROLLER_READY:
                    self._updateTime()
                    self._updatePerfFunc()

                    quat = self._FC.getQuat()
                    self._quat.append(np.array([quat.w, quat.x, quat.y, quat.z]))

                    # FRD body rate -> body-NED:
                    #   forward (FRD x)  ->  +x_body (roll rate)
                    #   right   (FRD y)  ->  +y_body (pitch rate)
                    #   down    (FRD z)  ->  +z_body (yaw rate) — sign flip for NED convention
                    w = self._FC.getAngVelIMU()
                    self._w.append(np.array([w.forward_rad_s, w.right_rad_s, -w.down_rad_s]))

                    feature_param = self._img_node.getImgFeatureParam()
                    opt_flow_ang_vel = self._img_node.getOptFlowAngVel()
                    # GT-FEEDBACK: substitute exact V-frame GT s/h (perception bypassed).
                    if self._gt_feedback is not None:
                        _p = self._pose_node.getPose()
                        if _p.UAV is not None and _p.target is not None:
                            _gt_fp, _gt_of = self._gt_feedback.update(
                                _p.UAV, _p.target, self._time.perf_counter())
                            _abl = self._gt_ablate
                            if not _abl:                      # full GT-FB (back-compat)
                                feature_param, opt_flow_ang_vel = _gt_fp, _gt_of
                            else:                             # per-channel: GT only the listed channels
                                feature_param = np.array(feature_param, dtype=float)
                                opt_flow_ang_vel = np.array(opt_flow_ang_vel, dtype=float)
                                if 's' in _abl:   feature_param[0:2]     = _gt_fp[0:2]
                                if 'yaw' in _abl: feature_param[3]       = _gt_fp[3]
                                if 'h' in _abl:   opt_flow_ang_vel[0:2]  = _gt_of[0:2]
                                if 'hz' in _abl:  opt_flow_ang_vel[2]    = _gt_of[2]
                                if 'wz' in _abl:  opt_flow_ang_vel[5]    = _gt_of[5]
                    self._updateImgFeatureParam(feature_param)
                    # Append _w_i BEFORE _updateOptFlow — the latter now uses
                    # self._w_i[-1] (MATLAB V_w) and would IndexError on the
                    # first iteration if appended after.
                    #
                    # Cap |w_i| at ±5 rad/s per axis. The raw lstsq + KF + the
                    # 1.98× sensor_cal_hw amplification on ω_z can produce
                    # transient |w_i| > 10 rad/s from image noise. That feeds
                    # cross(w_i, s) in _updateOptFlow / PLASMC, blowing up
                    # h_d → h_e → zeta → kappa → a_u → drone tumbles → more
                    # optic-flow noise → positive feedback. ±5 rad/s
                    # (~285 deg/s) is the X500's physical body-rate limit;
                    # values beyond this aren't physically meaningful for
                    # the marker's apparent relative angular velocity.
                    W_I_MAX = 5.0
                    _w_in = np.clip(opt_flow_ang_vel[3:], -W_I_MAX, W_I_MAX)
                    # ---- w_x,w_y de-rotation source (W_XY_DEROT) ----
                    # The image-derived roll/pitch angular flow w_x,w_y is weakly
                    # observable ONLY when corners CLUSTER near the image center
                    # (single small marker): the x^2,y^2,xy curvature that separates
                    # tilt-rate from translation vanishes, so the v_x<->w_y and
                    # v_y<->w_x columns go parallel. The multi-marker board + ring
                    # SPREAD restores that curvature -> w_x,w_y ARE recoverable
                    # (confirmed by a PROPER-EXCITATION re-test; the earlier r~-0.07
                    # "geometrically unobservable" result was an under-excitation
                    # artifact -- only ~0.1 rad/s achieved -- compounded by analysis
                    # in the V-LEVELED frame, which de-rotates r/p out by construction).
                    # Raw image w_x,w_y can still read noisy live (up to 3.4 rad/s; see
                    # Images/virtual_img_ang_vel.png) when r/p is unexcited; fed in it
                    # overdrives cross(V_w,S) -> a_u inflation -> body-rate saturation
                    # -> divergence (0 SP). w_z (yaw) IS well observed (swirl pattern)
                    # and is ALWAYS kept from the image.
                    #   'zero'  : w_x=w_y=0 — the validated b13/b14 config (2 SP/10).
                    #   'imu'   : w_x,w_y from the GYRO (accurate), transformed
                    #             body->V. For a STATIONARY target the camera's
                    #             angular velocity in V = Ry(pitch)Rx(roll)@omega_body
                    #             (level transform, compass-aligned V). Recovers the
                    #             real de-rotation the image cannot observe. Sign is
                    #             + (image-side overlays GT body rate, same phase).
                    #   'image' : keep the raw (broken) image w_x,w_y.
                    # MOVING/ROTATING targets need target omega added (not handled).
                    # Back-compat: CTRL_ZERO_WXY=1 (the old knob) maps to 'zero'.
                    _derot = os.environ.get(
                        "W_XY_DEROT",
                        "zero" if os.environ.get("CTRL_ZERO_WXY", "1") == "1" else "image")
                    if _derot == "imu":
                        roll, pitch, _yaw = Quaternion(self._quat[-1]).to_angles()
                        cr, sr = np.cos(roll), np.sin(roll)
                        cp, sp = np.cos(pitch), np.sin(pitch)
                        R_VB = np.array([[cp, sp * sr, sp * cr],
                                         [0.0, cr, -sr],
                                         [-sp, cp * sr, cp * cr]])   # Ry(pitch)@Rx(roll)
                        wb = self._w[-1]                              # [roll, pitch, -yaw_frd]
                        w_body_frd = np.array([wb[0], wb[1], -wb[2]]) # clean body-FRD omega
                        w_V = R_VB @ w_body_frd
                        _w_in = _w_in.copy(); _w_in[0] = w_V[0]; _w_in[1] = w_V[1]
                    elif _derot == "zero":
                        _w_in = _w_in.copy(); _w_in[0] = 0.0; _w_in[1] = 0.0
                    # 'image' -> leave _w_in as the raw image estimate
                    self._w_i.append(_w_in)
                    self._updateOptFlow(opt_flow_ang_vel[:3])

                    self._img_node.CONTROLLER_READY = True   # gates IMG_RECORD video to the descent
                    # Same gate for the external CHASE recorder (a separate
                    # process): touch the flag file once so record_chase.py starts
                    # recording at descent-start, matching the down-cam. Env-gated
                    # (no-op unless CHASE_GATE_FILE is set by the chase launch path).
                    if not getattr(self, "_chase_gated", False):
                        _cgf = os.environ.get("CHASE_GATE_FILE")
                        if _cgf:
                            try:
                                open(_cgf, "w").close()
                            except Exception:
                                pass
                        self._chase_gated = True
                    self.PLASMC()
                    self._yawCtrl()
                    self._attCtrl()

                if not self.TARGET_IS_VISIBLE:
                    # Archive the current log segment BEFORE the re-init wipes it, so the
                    # record runs continuously up to (and through) the controller failure.
                    if len(self._t) > 0:
                        self._log_segments.append(
                            {k: list(v) for k, v in self._buildLogDict().items()})
                    self._initialize_controller()
                    self.TARGET_IS_VISIBLE = True

            elif self.TARGET_IS_VISIBLE:
                self.TARGET_IS_VISIBLE = False

            time.sleep(SLEEP_TIME)

        if self._img_node.is_alive():
            self._img_node.close()
        self._img_node.join()

    def _updateTime(self):
        self._t.append(self._time.perf_counter())
        if len(self._t) > 1:
            while self._t[-1] == self._t[-2]:
                time.sleep(SLEEP_TIME)
                self._t[-1] = self._time.perf_counter()
            # ROOT-CAUSE FIX (2026-08-04, per user architectural correction -- see
            # project_pi_dt_visibility_independence_2026_08_04 memory, supersedes an
            # earlier dt-cap band-aid): dt used to be self._t[-1]-self._t[-2], i.e.
            # time since the LAST VISIBLE-marker tick -- since _updateTime() only
            # runs while FEATURE_IS_VISIBLE (see run()), ANY marker-loss gap (not
            # just the old RTL-handoff path -- ordinary in-grace coast bursts too,
            # documented elsewhere as commonly running up to ~300+ frames / several
            # seconds) showed up as a giant single-step dt feeding straight into
            # RK5/trapezoidal integrators on the reacquisition tick (root-caused:
            # u_a=-423497 rad/s for one tick, see
            # project_pi_yaw_rk5_dt_blowup_2026_08_04). The actual bug: time/dt
            # bookkeeping was WRONGLY coupled to marker visibility -- time passes
            # regardless of whether we can currently see the marker. Fixed at the
            # source instead of capped after the fact: dt is now taken from
            # self._last_loop_dt, refreshed EVERY outer-loop iteration in run()
            # independent of visibility, so it always reflects the true small
            # per-loop-iteration period (~SLEEP_TIME) -- never a visibility gap.
            # self._t itself is unchanged (still only appended on visible ticks,
            # preserving index-alignment with every other per-tick logged array).
            self._dt.append(self._last_loop_dt if self._last_loop_dt is not None
                             else self._t[-1] - self._t[-2])

    def _updatePerfFunc(self):
        """Middle-loop optical-flow performance envelope (MATLAB-style)."""
        t = self._t[-1] - self._t0
        decay = expm(-t * self._gamma)
        self._p.append(decay @ (self._p_0 - self._p_inf) + self._p_inf)
        self._dp.append(-self._gamma @ decay @ (self._p_0 - self._p_inf))
        if self._sen_funnel:
            # Outer-loop position funnel envelope on s_e_n (shrinking, same exp form)
            decay_s = expm(-t * self._gamma_s)
            self._p_s.append(decay_s @ (self._p_s_0 - self._p_s_inf) + self._p_s_inf)
            self._dp_s.append(-self._gamma_s @ decay_s @ (self._p_s_0 - self._p_s_inf))
        if self._combined_barrier:
            # Combined-barrier position funnel envelope p_r (FoV-consistent floor; Standing Cond 1)
            decay_r = expm(-t * self._xi_r)
            self._p_r.append(decay_r @ (self._p_r_0 - self._p_r_inf) + self._p_r_inf)
            self._dp_r.append(-self._xi_r @ decay_r @ (self._p_r_0 - self._p_r_inf))

    def _updateImgFeatureParam(self, s):
        """Outer loop: raw normalized pixel error -> PID -> desired feature derivative ds_d.

        MATLAB:
            V_s_e_n = (V_s - V_s_d) ./ p_10
            V_ds_d_xy = -rp*V_s_e_n - ri*iV_s_e_n - rd*dV_s_e_n
            V_ds_d = [V_ds_d_xy; 0]
        """
        self._s.append(s)
        if self._savgol_predict:
            self._s_raw.append(np.asarray(s, float).copy())      # IMMUTABLE raw measurement
            if len(self._s_raw) > 1 and len(self._dt) > 0:
                _dtc = max(float(self._dt[-1]), 0.006)
                dsn = np.abs((self._s_raw[-1][:2] - self._s_raw[-2][:2]) / (self._p_10 * _dtc))  # RAW |ds_n|
                spiked = dsn > self._ds_limit                    # PER-AXIS lateral (const s[2] never trips)
                self._s_good.append(not bool(np.any(spiked)))
                if np.any(spiked):
                    sc = np.asarray(self._s[-1], float).copy()
                    pred = (self._predictForward(self._s_raw, self._s_good, self._predict_lead)
                            if self._predict_mode == "savgol" else None)
                    for k in range(2):
                        if spiked[k]:
                            sc[k] = pred[k] if self._predict_mode == "savgol" else self._predictModel_s(k, self._predict_lead)
                    self._s[-1] = sc
            else:
                self._s_good.append(True)
        self._s_e.append(self._s[-1] - self._s_d)

        # Normalized pixel error (sensor-half normalization)
        s_e_n = self._s_e[-1][:2] / self._p_10
        # TERMINAL COMMIT latch: commit once the marker reliably fills the FoV. Trigger =
        # MEDIAN of MARKER_EXTENT_PX over the last _commit_win frames > threshold (the raw
        # extent is switching-noisy and SPIKES transiently — a 3-frame confirm let a spike
        # latch a spurious early commit @1.89 m → 10 m fly; the median filters those). s_e_n
        # STAYS LIVE (freezing it destabilizes, 0/3 variants); the committed flag instead
        # TERMINAL-CAPS the lateral command V_ds_d_xy below. Default-off (commit_extent=0).
        if self._commit_extent > 0 and not self._committed:
            self._ext_win.append(float(self.MARKER_EXTENT_PX))
            # Centered-gating: also require |s_e_n| < COMMIT_SEN so the commit fires only from a
            # centered state (else it coasts to an off-center offset -> 0/15 sub-meter). 0 = disabled.
            _centered = (self._commit_sen <= 0 or float(np.linalg.norm(s_e_n)) < self._commit_sen)
            if (len(self._ext_win) >= self._commit_win
                    and float(np.median(self._ext_win)) > self._commit_extent
                    and _centered):
                self._committed = True
                self._s_e_n_hold = s_e_n.copy()   # logged for diagnostics (offset committed at)
        self._s_e_n.append(s_e_n)
        # TERMINAL-COMMIT — "s = s_d" objective change. Ramp the POSITION error s_e_n -> 0, which makes
        # zeta_r -> 0 in BOTH paths consistently (surface sigma AND the h_d funnel-ref _hd_rate, both
        # built from s_e_n), CONTINUOUSLY, with chi_r FIXED (changing a surface coefficient would step
        # sigma -> discontinuous a_u). Rationale: post-commit we are over the target (marker leaving the
        # FoV) -> stop aligning, just land soft. The remaining surface sigma_xy = zeta_h(h_e) damps the
        # lateral velocity: h_e = h - h_d, and h is the MEASURED optical flow (lstsq, INDEPENDENT of the
        # centroid s), so post-commit (_hd_rate -> 0) h_e -> s_dot (residual bearing rate) and the SMC
        # drives it to 0. We zero s_e_n, NOT s itself, so that (a) h_d_noS keeps FF(s_actual) -> h_e =
        # s_dot exactly (no FF bias), and (b) dr_bar_e = s_dot/p_10 stays actual -> the chi_r*dzeta_r
        # residual-velocity term stays alive. (Overriding s would NOT hide the velocity -- h is measured
        # independently of s -- it would only bias the FF and kill the dzeta_r term.) Replaces the
        # surface-only zeta_r taper (which left the h_d path alive -> the residual kick).
        if self._terminal_commit and self._committed and self._tc_commit_t is not None:
            _a = min(1.0, (self._t[-1] - self._tc_commit_t) / self._tc_ramp_s) if self._tc_ramp_s > 0 else 1.0
            self._s_e_n[-1] = (1.0 - _a) * self._s_e_n[-1]
        # SOFT-BREACH source-fake (p_r): on a position-funnel breach, soft-clamp the BY-PRODUCT s_e_n to
        # FRAC*last-good HERE, at the source. Every downstream consumer (zeta_r barrier, outer PID, SEN-funnel,
        # integral, commit, h_d funnel-ref) then auto-adjusts — no per-consumer patch needed. Same pattern as
        # the commit ramp above (fake s_e_n, not s; h is measured independently of s). Flag drives the s_dot
        # source-fake below. p_r[-1] is available (funnel updated earlier).
        self._pr_breached = np.zeros(2, dtype=bool)
        if self._soft_breach and len(self._s_e_n) > 1:
            for _j in range(2):
                if abs(float(self._s_e_n[-1][_j]) / self._p_r[-1][_j]) >= 1.0 - S_MARGIN:
                    self._pr_breached[_j] = True
                    self._s_e_n[-1][_j] = self._soft_breach_frac * self._s_e_n[-2][_j]

        if self._combined_barrier:
            # === COMBINED-BARRIER: position barrier zeta_r enters the sliding surface (no back-map) ===
            # measured centroid rate s_dot (= d s_e[:2]/dt, s_d const) — the h_d feedforward AND
            # the r_bar_e rate (rel deg 2); kappa absorbs the 1/z-inflated s_ddot (dropped from dh_d).
            if len(self._s_e) > 1 and len(self._dt) > 0 and self._dt[-1] > 1e-6:
                self._s_dot_deque.append((self._s_e[-1][:2] - self._s_e[-2][:2]) / self._dt[-1])
            else:
                self._s_dot_deque.append(np.zeros(2))
            self._s_dot_deque.popleft()
            if self._vds_kf and len(self._dt) > 0 and self._dt[-1] > 1e-6:
                s_dot_meas = self._vdsKFStep(self._s_e[-1][:2], self._dt[-1])   # CV-KF velocity state
            else:
                s_dot_meas = smooth4(self._s_dot_deque)                          # MATLAB-parity finite-diff
            # SOFT-BREACH source-fake (p_r rate): soft-clamp the BY-PRODUCT s_dot on a position breach so
            # dr_bar_e -> zeta_r_dot -> the drift chi_r*zeta_r_dot (and _hd_rate) auto-adjust to the contained
            # rate instead of the raw 1/Z spike. Uses the p_r-breach flag set at the s_e_n source-fake.
            if self._soft_breach and getattr(self, "_pr_breached", None) is not None and len(self._s_dot_meas) > 0:
                for _j in range(2):
                    if self._pr_breached[_j]:
                        s_dot_meas[_j] = self._soft_breach_frac * self._s_dot_meas[-1][_j]
            self._s_dot_meas.append(s_dot_meas)
            # position barrier on r_bar_e = s_e_n (= s_e/p_10, FoV-normalized) with funnel p_r
            r_bar_e  = self._s_e_n[-1]
            dr_bar_e = s_dot_meas / self._p_10                       # measured rate (rel deg 2)
            S_r = np.zeros(2); zeta_r = np.zeros(2); dzeta_r = np.zeros(2)
            for _i in range(2):
                # s_e_n is already soft-clamped at the source on a p_r breach -> S_r auto-bounds here.
                S_r[_i] = float(np.clip(r_bar_e[_i] / self._p_r[-1][_i], -1.0 + S_MARGIN, 1.0 - S_MARGIN))
                zeta_r[_i]  = np.log((1 + S_r[_i]) / (1 - S_r[_i]))
                g_r         = (np.exp(zeta_r[_i]) + 1) ** 2 / (2 * np.exp(zeta_r[_i]) * self._p_r[-1][_i])
                dzeta_r[_i] = g_r * (dr_bar_e[_i] - S_r[_i] * self._dp_r[-1][_i])
            self._zeta_r.append(zeta_r)
            self._dzeta_r.append(dzeta_r)
            # no back-mapped ds_d in combined mode (h_d uses s_dot_meas directly)
            self._ds_d.append(np.zeros(N_DIM))
            return

        if self._sen_funnel:
            # === PPC funnel on s_e_n (back-mapped form; mirrors baseline :256-289) ===
            # Mirror the optic-flow funnel breach handling (lines ~581-583): clamp ONLY
            # the ratio for log-finiteness and leave the stored s_e_n untouched. Same
            # logic in both PPC funnels -- a breach is absorbed by the clamp, not by
            # rewriting the measurement.
            S_s = np.eye(2); zeta_s = np.zeros(2); G_s = np.eye(2)
            for idx in range(2):
                r = self._s_e_n[-1][idx] / self._p_s[-1][idx]
                r = float(np.clip(r, -1.0 + S_MARGIN, 1.0 - S_MARGIN))
                S_s[idx, idx] = r
                zeta_s[idx] = np.log((1 + r) / (1 - r))
                G_s[idx, idx] = (np.exp(zeta_s[idx]) + 1) ** 2 / (2 * np.exp(zeta_s[idx]) * self._p_s[-1][idx])
            self._S_s.append(S_s); self._zeta_s.append(zeta_s); self._G_s.append(G_s)
            # integral of zeta_s (trapezoidal, anti-windup via izeta_clamp)
            if len(self._izeta_s) == 0:
                self._izeta_s.append(np.zeros(2))
            else:
                ni = (self._izeta_s[-1]
                      + self._dt[-1] * 0.5 * (self._zeta_s[-1] + self._zeta_s[-2]))
                ni = np.clip(ni, -self._izeta_clamp, self._izeta_clamp)
                self._izeta_s.append(ni)
            # smoothed derivative of zeta_s
            if len(self._zeta_s) > 1:
                self._dzeta_s_deque.append((self._zeta_s[-1] - self._zeta_s[-2]) / self._dt[-1])
                self._dzeta_s_deque.popleft()
            dzeta_s = smooth4(self._dzeta_s_deque)
            # desired barrier dynamics (reuse PID gains) -> back-map + envelope-track
            dzeta_sd = (- self._K_rp @ self._zeta_s[-1]
                        - self._K_ri @ self._izeta_s[-1]
                        - self._K_rd @ dzeta_s)
            V_ds_d_xy = np.linalg.inv(G_s) @ dzeta_sd + S_s @ self._dp_s[-1]
        else:
            # === legacy outer PID on s_e_n (default) ===
            # Trapezoidal integration of normalized error + anti-windup, with
            # CONDITIONAL INTEGRATION (freeze, mirrors _yawCtrl's ie_a anti-windup,
            # 2026-06-08): a hard magnitude clamp bounds izeta's VALUE but does not stop
            # it accumulating a fictitious error during a genuine feature coast (real
            # 2026-07-30 hardware flights: s_e_n dead-reckoned via h_extrap/map_flow
            # ramps linearly for ~400 frames while cbf_corners/FEATURE_PTS_FRESH are
            # unfresh -> is_e_n/izeta wind up to their clamp on a signal that isn't real
            # tracking error). Gate on FEATURE_PTS_FRESH (raw decode OR a validated
            # rescue succeeded THIS frame), NOT FEATURE_IS_STALE -- Gazebo's own
            # 2026-07-17 cbf_corners fix (see CBF_CORNERS_STALE docstring) found
            # FEATURE_IS_STALE is a legacy raw-decode-miss counter with no rescue
            # awareness that trips after just 3 misses even while a rescue is
            # successfully covering every one -- gating on it would blind this
            # integral during exactly the window a working rescue is active.
            _feat_fresh = bool(getattr(self._img_node, "FEATURE_PTS_FRESH", True))
            if len(self._is_e_n) == 0:
                self._is_e_n.append(np.zeros(2))
            elif not _feat_fresh:
                self._is_e_n.append(self._is_e_n[-1].copy())   # hold -- do not integrate while unfresh
            else:
                new_int = (self._is_e_n[-1]
                           + self._dt[-1] * 0.5 * (self._s_e_n[-1] + self._s_e_n[-2]))
                n = np.linalg.norm(new_int)
                if n > self._iV_s_e_n_clamp:
                    new_int = new_int * (self._iV_s_e_n_clamp / n)
                self._is_e_n.append(new_int)
            # Smoothed derivative of normalized error
            if len(self._s_e_n) > 1:
                self._ds_e_n_deque.append((self._s_e_n[-1] - self._s_e_n[-2]) / self._dt[-1])
                self._ds_e_n_deque.popleft()
            ds_e_n = smooth4(self._ds_e_n_deque)
            # PID -> desired feature-time-derivative (manuscript form, no clamps)
            V_ds_d_xy = (- self._K_rp @ self._s_e_n[-1]
                         - self._K_ri @ self._is_e_n[-1]
                         - self._K_rd @ ds_e_n)

        # --- escalating recovery authority on funnel breach (PLASMC_SEN_RECOVERY_K) ---
        # The back-mapped barrier demand g(r)=(1-r^2)*log((1+r)/(1-r)) PEAKS at r~0.65
        # and COLLAPSES toward the boundary; the ratio clamp (|r|<=0.95) then FREEZES
        # zeta_s, so once |s_e_n|>p_s the outer loop has NO escalating authority to pull
        # the feature back -- the overshoot breach is structurally unrecoverable (the
        # demand at r>3 is WEAKER than at r=0.65; see the s_e/p_s breach analysis).
        # Add a linear recovery demand that activates on breach and GROWS with the
        # overflow (|s_e_n|-p_s), restoring inward authority that escalates with breach
        # severity (the signal the ratio clamp discards). Acts in feature-rate units,
        # same as ds_d (scale-free, no Z). Default 0.0 = OFF (A/B + IC2-5 gated).
        if self._sen_recovery_k > 0.0:
            for _i in range(2):
                over = abs(self._s_e_n[-1][_i]) - self._p_s[-1][_i]
                if over > 0.0:
                    V_ds_d_xy[_i] += -self._sen_recovery_k * over * np.sign(self._s_e_n[-1][_i])

        # Lateral approach-velocity governor: cap the closing feature-rate demand
        # so the drone doesn't command (and build) a lateral velocity it can't see.
        n_xy = float(np.linalg.norm(V_ds_d_xy))
        if n_xy > self._DSD_LAT_MAX:
            V_ds_d_xy = V_ds_d_xy * (self._DSD_LAT_MAX / n_xy)
        # TERMINAL COMMAND CAP (command-bounding, keeps s_e_n LIVE/consistent): once committed
        # (marker fills FoV), bound the lateral feature-rate demand so the 1/Z-amplified s_e_n
        # can't whip the drone laterally → marker-loss fly-away. Unlike freezing s_e_n, the loop
        # still SEES the real error (direction preserved, magnitude capped) → no controller
        # inconsistency. PLASMC_COMMIT_DSD_MAX=0 → OFF (the cap only acts when commit_extent>0).
        if self._committed and self._commit_dsd_max > 0 and n_xy > self._commit_dsd_max:
            V_ds_d_xy = V_ds_d_xy * (self._commit_dsd_max / n_xy)
        raw_ds_d = np.concatenate([V_ds_d_xy, [0.0]])
        if self._tau_ds > 0 and len(self._dt) > 0:
            if not self._ds_d_lpf_init:
                self._ds_d_lpf = raw_ds_d.copy()
                self._ds_d_lpf_init = True
            else:
                alpha = self._dt[-1] / (self._tau_ds + self._dt[-1])
                self._ds_d_lpf += alpha * (raw_ds_d - self._ds_d_lpf)
            self._ds_d.append(self._ds_d_lpf.copy())
        else:
            self._ds_d.append(raw_ds_d)

    def _updateOptFlow(self, h):
        """Middle-loop: barrier-transform optical flow error, prep zeta / sigma inputs."""
        # TERMINAL RING-COMMIT flow override (one-frame lag off _ringCommitStep, a one-way latch):
        # h_xy->0 (centered+settled => true lateral ~0; ring lateral is unobservable) and
        # h_z->marker-less ring loom (the flickering/staling marker moment-loom is retired). s / zeta_r
        # (position) are untouched -> the marker position loop keeps steering (moving-target-OK).
        if self._ring_committed:
            h = np.array(h, dtype=float)
            h[0] = 0.0; h[1] = 0.0
            h[2] = float(getattr(self._img_node, 'RING_LOOM', h[2]))
        elif self._loom_ring_on_loss:
            # SIMPLE loom-only rule: terminal decode lost (observer inactive) + marker NOT drifted off
            # (overflow/occlusion, still over target) -> h_z <- ring loom; h_xy left as-is.
            _lost = not bool(getattr(self._img_node, '_observer_valid', True))
            _over = bool(getattr(self._img_node, 'OVER_TARGET', False))   # nadir inside small marker = terminal gate (rejects the altitude flicker)
            if _lost and _over and not bool(getattr(self._img_node, '_last_drifted_off', False)):
                _rl = float(getattr(self._img_node, 'RING_LOOM', h[2]))
                if os.environ.get("LOOM_RING_DBG", "0") == "1":
                    _nst = int(getattr(self._img_node, 'RING_N_STATIONS', -1))
                    _rdv = float(getattr(self._img_node, 'RING_DIV_RAW', float('nan')))
                    _rmm = float(getattr(self._img_node, 'RING_MOMENT_RAW', float('nan')))
                    print("[lr] t%.3f LOOM->RING: marker_hz=%+.2f ring_hz=%+.2f (src=%s) "
                          "div_raw=%+.2f mom_raw=%+.2f Nstations=%d over=%s drifted=%s" % (
                        float(getattr(self._img_node, '_stamp', 0.0)), float(h[2]), _rl,
                        self._img_node._ring_loom_source, _rdv, _rmm, _nst, _over,
                        bool(getattr(self._img_node, '_last_drifted_off', False))), flush=True)
                h = np.array(h, dtype=float)
                h[2] = _rl
        self._h.append(h)
        if self._savgol_predict:
            self._h_raw.append(np.asarray(h, float).copy())      # IMMUTABLE raw measurement
            if len(self._h_raw) > 1 and len(self._dt) > 0:
                _dtc = max(float(self._dt[-1]), 0.006)           # floor tiny/duplicate-stamp dt
                dh = np.abs((self._h_raw[-1] - self._h_raw[-2]) / _dtc)   # gate on RAW dh (terminal-selective)
                spiked = dh > self._dh_limit                     # PER-AXIS (loom kept in the good-mask)
                self._h_good.append(not bool(np.any(spiked)))    # frame good iff no axis spiked
                if np.any(spiked[:2]):                           # LATERAL spike -> reconstruct lateral only
                    hc = np.asarray(self._h[-1], float).copy()
                    _hc_before = hc.copy()
                    pred = (self._predictForward(self._h_raw, self._h_good, self._predict_lead)
                            if self._predict_mode == "savgol" else None)
                    for k in range(2):                           # LATERAL ONLY. The loom (k=2) is LEFT as the
                        if spiked[k]:                            # genuine measurement: its dynamics
                            hc[k] = (pred[k] if self._predict_mode == "savgol"   # ḣ_z=-β·a_z-h_z² (manuscript
                                     else self._predictModel_h(k, self._predict_lead))  # eq h-dot) are β-coupled,
                    self._h[-1] = hc                             # not depth-free predictable; the z-funnel must see it
                    if os.environ.get("SAVGOL_PREDICT_DBG", "0") == "1":
                        print("[sgp] t%.3f SPIKE-RECONSTRUCT: dh=%s spiked=%s mode=%s h_raw_prev=%s h_raw_now=%s "
                              "h_before=%s h_after=%s" % (
                            float(getattr(self._img_node, '_stamp', 0.0)), np.round(dh, 3), spiked.tolist(),
                            self._predict_mode, np.round(self._h_raw[-2], 3), np.round(self._h_raw[-1], 3),
                            np.round(_hc_before, 3), np.round(hc, 3)), flush=True)
            else:
                self._h_good.append(True)

        # LEVER 2: blend the accurate centroid-rate into the lateral flow (see __init__).
        # d(s[:2])/dt = the V-frame translational flow, free of the LK ~1 rad/s ceiling and
        # the lstsq garbage spikes that make h noisy. Replaces ONLY h[:2] (lateral); h[2]
        # (loom) + h[3:] (rotation) keep the LK/lstsq value. Default off (blend=0).
        if (self._FLOW_CENTROID_RATE > 0.0 and len(self._s) > 1
                and len(self._dt) > 0 and self._dt[-1] > 1e-6):
            _s_rate = (self._s[-1][:2] - self._s[-2][:2]) / self._dt[-1]
            self._s_rate_deque.append(_s_rate); self._s_rate_deque.popleft()
            _b = self._FLOW_CENTROID_RATE
            self._h[-1][:2] = (1.0 - _b) * self._h[-1][:2] + _b * smooth4(self._s_rate_deque)

        # Desired optical flow — MATLAB form (visualControl_IBVS_adaptive.m:369-370).
        # MATLAB uses V_w (optical-flow-derived target-relative ang vel in V frame),
        # NOT the UAV body rate. The corresponding Python variable is self._w_i
        # (= getOptFlowAngVel()[3:] = KF-smoothed lstsq output, V-frame). Using
        # self._w (body IMU) here was a parity bug — frames don't match (V vs body)
        # and physical meaning differs (target-relative vs absolute).
        w = self._w_i[-1]
        # MATLAB visualControl_IBVS_adaptive.m:369-370 EXACTLY:
        #   V_h_d = V_ds_d + cross(V_w, V_s(1:3))
        #         + (h_rd - dot(cross(V_w, V_s(1:3)), e3)) * V_s(1:3)
        # The earlier Python had BOTH coupling-term signs flipped (−cross and
        # +dot). On z that cancels to h_rd at s≈[0,0,1] so descent worked, but
        # x/y picked up doubled cross-coupling — V_h_d[0,1] excursions hit ±20
        # vs MATLAB's ±4, over-driving the SMC and giving 8× MATLAB descent rate.
        # Direct h_ref (MATLAB-equivalent). Previously had a soft-engage ramp
        # and a lateral-error gate here — both removed (see __init__ note).
        h_ref_eff = self._h_ref
        if self._descent_gate and len(self._s_e_n) > 0:
            sen = float(np.linalg.norm(self._s_e_n[-1]))
            if sen <= self._dgate_slo:
                g_t = 1.0
            elif sen >= self._dgate_shi:
                g_t = self._dgate_gmin
            else:
                u = (sen - self._dgate_slo) / max(self._dgate_shi - self._dgate_slo, 1e-6)
                g_t = 1.0 - (u * u * (3.0 - 2.0 * u)) * (1.0 - self._dgate_gmin)   # smoothstep
            # rate-limit g via 1-pole LPF -> bounded d(h_ref)/dt -> no destabilizing dh_d transient
            _dt = self._dt[-1] if (len(self._dt) > 0 and self._dt[-1] > 1e-6) else 0.008
            self._dgate_g += (_dt / max(self._dgate_tau, _dt)) * (g_t - self._dgate_g)
            h_ref_eff = self._h_ref * self._dgate_g
        cross_ws = np.cross(w, self._s[-1][:3])
        if self._combined_barrier:
            # blended surface: h_d = MEASURED s_dot + transport + descent (NO back-mapped ds_d).
            # h_d_noS (transport+descent only) feeds dh_d so s_ddot is dropped (kappa absorbs it).
            # transport/descent follow the c_h convention (CH_CLEAN = clean-IMU psi_dot_b, else w_i x s),
            # matching the PLASMC c-term so h_d and c stay consistent.
            if self._CH_CLEAN:
                _eul = Quaternion(self._quat[-1]).to_angles()
                _phi, _th = float(_eul[0]), float(_eul[1])
                _r = -float(self._w[-1][2])
                psi_dot_b = (float(self._w[-1][1]) * np.sin(_phi) + _r * np.cos(_phi)) / max(abs(np.cos(_th)), 0.2)
                rot = psi_dot_b * np.cross(e3, self._s[-1][:3])
                h_d_ff = h_ref_eff * self._s[-1][:3]
            else:
                rot = cross_ws
                h_d_ff = (h_ref_eff - np.dot(cross_ws, e3)) * self._s[-1][:3]
            h_d_noS = rot + h_d_ff
            self._h_d_noS.append(h_d_noS)
            if self._hd_passive:
                # Clean stacked-barrier design: NO desired-rate term. h_d = passive FF (h_d_noS) only,
                # so the non-vanishing surface term chi_r*zeta_r is the sole s_e_n driver.
                _hd_rate = np.zeros(2)
                _hd_rate_smooth = _hd_rate
            elif self._hd_funnel_ref:
                # back-map V_ds_e = inv(g_r)*dzeta_rd + S_r*dp_r, dzeta_rd = -k_r*zeta_r (proportional
                # prescription zeta_r_dot = -k_r*zeta_r). p_10 un-normalizes to s_dot_meas (= ds_e) units.
                # k_r=0 -> funnel-only (S_r*dp_r); k_r>0 adds the convergence/recovery term -k_r*zeta_r/g_r.
                _S_r = np.clip(self._s_e_n[-1][:2] / self._p_r[-1], -1.0 + S_MARGIN, 1.0 - S_MARGIN)
                _hd_rate_smooth = self._p_10 * _S_r * self._dp_r[-1]
                if self._hd_kr != 0.0:
                    _zeta_r = self._zeta_r[-1]
                    _g_r = (np.exp(_zeta_r) + 1.0) ** 2 / (2.0 * np.exp(_zeta_r) * self._p_r[-1])
                    _hd_rate = _hd_rate_smooth - self._p_10 * self._hd_kr * _zeta_r / _g_r
                else:
                    _hd_rate = _hd_rate_smooth
            else:
                _hd_rate = self._s_dot_meas[-1]
                _hd_rate_smooth = _hd_rate
            self._h_d.append(np.concatenate([_hd_rate, [0.0]]) + h_d_noS)
            self._h_d_kfree.append(np.concatenate([_hd_rate_smooth, [0.0]]) + h_d_noS)
            self._hd_rate_log.append(np.asarray(_hd_rate, dtype=float).copy())   # DIAG: log the rate term
        elif self._CH_CLEAN:
            # Consistent c_h kinematics correction (manuscript §II, 2026-06-11; feedback_ch_kinematics_correction).
            # Desired flow DROPS the w×s rotational feedforward — the corrected c-term carries the rotation
            # via -ψ̇_b·(ê3×h) instead (see PLASMC). Removing w×s here is the OTHER half of the consistent
            # change (MATLAB C_SIMPLE only changed c -> convention-mixed regression; this does both). Directly
            # de-dominates h_d (which the a_u-outward diagnosis showed is ~all cross(w_i,s) in the overshoot).
            self._h_d.append(self._ds_d[-1] + h_ref_eff * self._s[-1][:3])
            self._h_d_kfree.append(self._h_d[-1])       # no k_r branch outside combined mode
        else:
            self._h_d.append(
                self._ds_d[-1]
                + cross_ws
                + (h_ref_eff - np.dot(cross_ws, e3)) * self._s[-1][:3]
            )
            self._h_d_kfree.append(self._h_d[-1])       # no k_r branch outside combined mode
        self._h_e.append(self._h[-1] - self._h_d[-1])

        # Barrier transform on h_e — MATLAB visualControl_IBVS_adaptive.m:380-385.
        # Only the RATIO is clamped (for log finiteness); the stored h is left
        # untouched so the downstream c-term still sees the actual measurement.
        S = np.eye(N_DIM)
        zeta = np.zeros(N_DIM)
        G = np.eye(N_DIM)
        contained = np.zeros(N_DIM, dtype=bool)   # True on axes where outlier containment fired
        # SINGHAL-2025 OUTLIER CONTAINMENT (their controller.py SMC). A flow error beyond the funnel
        # (|h_e/p| >= 1) is a perception OUTLIER (e.g. a loom glitch h_z -0.3 -> -8, held across
        # several control steps until the next image frame). HOLD the last-good eta (sign-adjusted)
        # and RECONSTRUCT h_e and h onto the funnel boundary, so the glitch can't leak into the
        # c-term's quadratic h-feed-forward (dot(h,e3)*h ~ 64) -> a_u explosion (-2000) -> 54 m crash.
        # (A |Δh| rate-gate was tried 2026-06-08 and REVERTED: the glitch is held for several control
        # steps with Δh=0, so the rate gate missed the held frames -> explosions returned. The
        # funnel-bound trigger catches every glitch-affected frame.) 2026-06-08.
        for idx in range(N_DIM):
            ratio = self._h_e[-1][idx] / self._p[-1][idx]
            if abs(ratio) >= 1.0 and len(self._S) > 0:
                if self._soft_breach:                      # IDEA 1: pull to FRAC of last in-funnel value (off the singular edge); keep kappa adapting
                    ratio = self._soft_breach_frac * float(self._S[-1][idx, idx])
                    self._h_e[-1][idx] = ratio * self._p[-1][idx]
                    self._h[-1][idx] = self._h_e[-1][idx] + self._h_d[-1][idx]
                    # NO contained[idx]=True: feature is off the edge -> G bounded -> kappa_eq bounded -> kappa adapts safely (no freeze/no runaway)
                else:
                    if self._contain_hold_full and idx < 2:   # lateral: hold full last-good (no sign-flip feed)
                        ratio = float(self._S[-1][idx, idx])
                    else:                                      # legacy / z: hold magnitude, trust current sign
                        ratio = np.abs(float(self._S[-1][idx, idx])) * np.sign(ratio)
                    self._h_e[-1][idx] = ratio * self._p[-1][idx]             # reconstruct contained h_e
                    self._h[-1][idx] = self._h_e[-1][idx] + self._h_d[-1][idx]  # and h (used by c-term)
                    contained[idx] = True
            ratio = float(np.clip(ratio, -1.0 + S_MARGIN, 1.0 - S_MARGIN))
            S[idx, idx] = ratio
            zeta[idx] = np.log((1 + ratio) / (1 - ratio))
            G[idx, idx] = (np.exp(zeta[idx]) + 1) ** 2 / (2 * np.exp(zeta[idx]) * self._p[-1][idx])
        self._S.append(S)
        self._zeta.append(zeta)
        self._G.append(G)
        self._contained = contained   # used by κ-ODE: freeze adaptation on outlier axes

        # Smoothed derivative of desired optical flow, with physical cap.
        # Without the cap, the PID's first non-zero firing produces a step
        # in V_h_d that gives raw dh_d ≈ (Δh_d)/dt ≈ 60-160 m/s² — feeds the
        # c-term, blows up |a_u|, drone flies away. Real-flight |dh_d| is
        # well under 5 m/s² even during aggressive maneuvers.
        #
        # 2026-06-03 cleanup REVERTED same day: the 14:20 refactor restored 50.0
        # assuming the convergence-ordering gains make the κ-runaway patch
        # unnecessary. Empirically false — LateralRestore c1 (identical gains,
        # only this default differing) went 4/4 catastrophic vs b13/b14's ~35%
        # tail (test_data/LateralRestore/*_ABORTED_dhdmax50_evidence). The
        # validated 28%-SP config was gated and approved WITH 5.0 baked
        # (commit 2b43983, IC2-5 gate passed); it is load-bearing until the
        # 1/Z touchdown spike has a manuscript-parameter fix.
        DH_D_MAX = self._DH_D_MAX
        # In combined mode, differentiate h_d_noS (transport+descent) — drops s_ddot (kappa absorbs it).
        # funnel-ref: the S_r*dp_r rate part is SMOOTH -> carried honestly; the -k_r*zeta_r/g_r branch
        # (baked 06-29) is BARRIER-INFLATED (s_ddot-class; ~half of c3's rover-curve cycle content at
        # ~1.7 rad/s) -> PLASMC_DHD_SRC picks full (06-29) / nokr (drop k_r branch only) / nos (drop all).
        if self._combined_barrier:
            # passive: h_d == h_d_noS (rate=0) -> differentiate h_d_noS (no s_ddot to absorb anyway)
            if self._hd_funnel_ref and not self._hd_passive:
                _hd_src = (self._h_d if self._dhd_src == "full"
                           else self._h_d_kfree if self._dhd_src == "nokr"
                           else self._h_d_noS)
            else:
                _hd_src = self._h_d_noS
        else:
            _hd_src = self._h_d
        if len(_hd_src) > 1:
            self._dh_d_deque.append((_hd_src[-1] - _hd_src[-2]) / self._dt[-1])
            self._dh_d_deque.popleft()
        if self._dhd_kf and len(_hd_src) > 0 and len(self._dt) > 0 and self._dt[-1] > 1e-6:
            _dhd = self._cvkfVecRate(self._dhd_kf_st, _hd_src[-1], self._dt[-1])   # CV-KF rate of the selected _hd_src
        else:
            _dhd = smooth4(self._dh_d_deque)                                       # MATLAB-parity finite-diff
        self._dh_d.append(np.clip(_dhd, -DH_D_MAX, DH_D_MAX))

    def PLASMC(self):
        """Middle-loop adaptive SMC -> body-frame acceleration command a_u."""
        t = self._t[-1] - self._t0

        # Terminal-commit discriminator — evaluated HERE (not in _updateImgFeatureParam) so h_z is
        # the CURRENT frame's loom (_updateOptFlow has run; _h[-1] is fresh, no one-frame lag). Sets
        # _committed BEFORE the surface assembly below so the zeta_r-zeroing acts this same step.
        # ds_e_n = s_dot_meas/p_10 (filtered, combined-mode only); s_e_n = last normalized error.
        if (self._combined_barrier and len(self._s_e_n) > 0 and len(self._s_dot_meas) > 0):
            _dsen = self._s_dot_meas[-1] / self._p_10
            if self._terminal_ring_commit:          # retargeted: swap velocity source, keep zeta_r
                self._ringCommitStep(self._s_e_n[-1], _dsen)
            elif self._terminal_commit:             # legacy: zero zeta_r (disabled by default)
                self._terminalCommitStep(self._s_e_n[-1], _dsen)

        # Loom-inversion touchdown detector (depth-free). h_z is fresh (_updateOptFlow ran); s_e_n latest.
        if self._touchdown_loom and len(self._s_e_n) > 0 and len(self._h) > 0:
            self._touchdownDetect(self._s_e_n[-1])

        # Integral of zeta (trapezoidal) with anti-windup, CONDITIONAL INTEGRATION
        # (freeze while the feature measurement feeding zeta is unfresh -- see the
        # matching is_e_n comment above; same 2026-07-30 hardware finding, same
        # FEATURE_PTS_FRESH gate, same reason FEATURE_IS_STALE is the wrong flag).
        _feat_fresh = bool(getattr(self._img_node, "FEATURE_PTS_FRESH", True))
        if len(self._izeta) == 0:
            self._izeta.append(np.zeros(N_DIM))
        elif not _feat_fresh:
            self._izeta.append(self._izeta[-1].copy())   # hold -- do not integrate while unfresh
        else:
            new_int = (self._izeta[-1]
                       + self._dt[-1] * 0.5 * (self._zeta[-1] + self._zeta[-2]))
            # Anti-windup: per-COMPONENT clamp (matches MATLAB visualControl_IBVS
            # _adaptive.m:393-394). Was norm-clamp, which saturated ~30% earlier
            # on 3-vectors at the limit (norm=√3·5 vs per-axis 5 each).
            new_int = np.clip(new_int, -self._izeta_clamp, self._izeta_clamp)
            self._izeta.append(new_int)
        # Option-A: reset izeta_xy to 0 at the FIRST committed step so the post-commit PI integral
        # references the centered commit position (no carried-over approach accumulation/windup).
        if (self._tc_integral and self._committed and not self._tc_izeta_reset_done):
            self._izeta[-1][0] = 0.0
            self._izeta[-1][1] = 0.0
            self._tc_izeta_reset_done = True

        # Angular acceleration: smoothed derivative of V_w_i (MATLAB derives V_dw from V_w_i,
        # visualControl_IBVS_adaptive.m:295-299). V_w_i is FRAME-HELD — img_data updates it ~42 Hz
        # while control runs ~125 Hz. Cleaner dw (2026-06-10): on a NEW frame, divide the frame-jump
        # by the REAL inter-frame interval (not control_dt → removes the ~3× over-amplification that
        # made cross(dw,s)→θ_norm spike, |dw|=252); BETWEEN frames the numerator is 0 so dw is 0 — it
        # stays a BRIEF spike, NOT held (a held dw poisons the κ-integrator with sustained moderate θ:
        # tested + reverted). Plus a physical |dw| clamp. NOTE: this only tidies the θ PEAK; the κ
        # runaway is the SUSTAINED θ, ~74% the non-dw c-terms — handled by the close-range κ-freeze.
        if len(self._w_i) > 1:
            if not np.allclose(self._w_i[-1], self._w_i[-2]):
                dt_frame = (self._t[-1] - self._w_i_last_t) if self._w_i_last_t is not None else self._dt[-1]
                self._w_i_last_t = self._t[-1]
            else:
                dt_frame = self._dt[-1]      # irrelevant: numerator is 0 between frames
            dw_raw = (self._w_i[-1] - self._w_i[-2]) / max(dt_frame, 1e-3)
            self._dw_deque.append(np.clip(dw_raw, -self._dw_max, self._dw_max))
            self._dw_deque.popleft()
        if self._dw_kf and len(self._w_i) > 0 and len(self._dt) > 0 and self._dt[-1] > 1e-6:
            # CV-KF on w_i → rate; spreads the frame-held stair-step jump instead of spiking it
            self._dw.append(np.clip(self._cvkfVecRate(self._dw_kf_st, self._w_i[-1], self._dt[-1]),
                                    -self._dw_max, self._dw_max))
        else:
            self._dw.append(smooth4(self._dw_deque))

        # Sliding surface. Back-mapped (default): sigma = zeta_h + Omega*int(zeta_h).
        # Combined-barrier: lateral sigma_k = zeta_h_k + chi_r*zeta_r_k (PD), descent
        # sigma_3 = zeta_h_3 + chi_z*int(zeta_h_3) (PI, chi_z = Omega_z, unchanged). chi_zeta_aug =
        # chi*zeta_dot_aug is the measured drift folded into u_eq/Theta (= Omega*zeta_h back-mapped;
        # = [chi_r*dzeta_r; Omega_z*zeta_h_3] combined). Relative degree 1 preserved.
        if self._combined_barrier:
            _zr  = self._zeta_r[-1]
            _dzr = self._dzeta_r[-1]
            # TERMINAL-COMMIT: zeta_r (and its rate) are zeroed AT THE SOURCE via the s_e_n->0 ramp in
            # _updateImgFeatureParam — zeta_r is built from s_e_n, so it -> 0 in BOTH this surface AND
            # the h_d funnel-ref _hd_rate CONSISTENTLY and continuously, with chi_r FIXED. So _zr -> 0
            # here automatically; _dzr -> g_r*dr_bar_e (built from the ACTUAL bearing rate) stays live,
            # so chi_r*dzeta_r in chi_zeta_aug -> a residual-VELOCITY damping term. Replaces the old
            # surface-only taper (which faded _zr/_dzr here but left zeta_r alive in the h_d path).
            _sig = self._zeta[-1].copy()
            _sig[0] += self._chi_r[0] * _zr[0]
            _sig[1] += self._chi_r[1] * _zr[1]
            _sig[2] += self._Omega[2, 2] * self._izeta[-1][2]
            chi_zeta_aug = np.array([self._chi_r[0] * _dzr[0],
                                     self._chi_r[1] * _dzr[1],
                                     self._Omega[2, 2] * self._zeta[-1][2]])
            # Option-A: post-commit, the faded chi_r*zeta_r is replaced by a PI-on-flow integral
            # Omega_xy*int(zeta_h_xy) (sigma_xy = zeta_h_xy + Omega_xy*izeta_xy), mirroring z. Drift
            # fold = Omega_xy*zeta_h_xy (= d/dt of the integral term), as for z.
            if self._tc_integral and self._committed:
                _sig[0] += self._tc_omega_xy * self._izeta[-1][0]
                _sig[1] += self._tc_omega_xy * self._izeta[-1][1]
                chi_zeta_aug[0] += self._tc_omega_xy * self._zeta[-1][0]
                chi_zeta_aug[1] += self._tc_omega_xy * self._zeta[-1][1]
            self._sigma.append(_sig)
        else:
            self._sigma.append(self._zeta[-1] + self._Omega @ self._izeta[-1])
            chi_zeta_aug = self._Omega @ self._zeta[-1]

        # Known dynamics term c — MATLAB visualControl_IBVS_adaptive.m:407-408.
        # All cross products use V-frame target-relative ω (self._w_i), matching
        # MATLAB's V_w. Self._w (body IMU rate) is logged but not used here.
        w = self._w_i[-1]
        if self._CH_CLEAN:
            # CONSISTENT c_h CORRECTION (manuscript §II; feedback_ch_kinematics_correction).
            # New: c = -ψ̇_b·(ê3×h) - (h·ê3)·h - ḣ_d  (clean IMU-yaw frame rotation + loom only;
            # NO ẇ×s / w×(w×s) / 2w×h — the noisy V_w/V_dw cross-products the old form transplanted
            # from camera-frame static-target kinematics). Paired with the w×s drop in h_d above.
            # ψ̇_b = ZYX yaw Euler rate from the body rate (transport term collapses to ψ̇ ê3×h in the
            # gravity-leveled V-frame). ⚠ ψ̇_b sign/convention is SITL-unverified — if the A/B regresses,
            # flip the _r sign first. MATLAB C_SIMPLE (partial) regressed nominal SP 9->2 (the w-terms do
            # real FF work) — this consistent form is the fair test; DEFAULT-OFF, A/B before trusting.
            _eul = Quaternion(self._quat[-1]).to_angles()        # [roll, pitch, yaw]
            _phi, _th = float(_eul[0]), float(_eul[1])
            _r = -float(self._w[-1][2])                          # true FRD yaw rate (self._w stores -down)
            # PLASMC_CH_PSIDOT_SIGN flips the transport-term yaw rate (default +1). The clean-form
            # A/B regressed (IC2 58m, diverges earlier+harder = ANTI-RESTORING, not lost FF) — the
            # psi_dot_b sign in -psi_dot_b*(e3 x h) is SITL-unverified (memory flag) and the GT-FB
            # w_z is itself +psidot_b vs the manuscript -psidot_b, so the transport term likely pushes
            # the wrong way. Set =-1 to flip and retest (the memory's "flip the _r sign first" remedy).
            psi_dot_b = self._ch_psidot_sign * (float(self._w[-1][1]) * np.sin(_phi) + _r * np.cos(_phi)) / max(abs(np.cos(_th)), 0.2)
            c = (- psi_dot_b * np.cross(e3, self._h[-1])
                 - self._cterm_loom_scale * np.dot(self._h[-1], e3) * self._h[-1]
                 - self._dh_d[-1])
        else:
            _dws = np.cross(self._dw[-1], self._s[-1][:3])   # omega_dot x s — angular-accel feedforward
            if self._cterm_dws_max > 0.0:                    # cap its magnitude (terminal-launch guard)
                _ndws = float(np.linalg.norm(_dws))
                if _ndws > self._cterm_dws_max:
                    _dws = _dws * (self._cterm_dws_max / _ndws)
            c = (_dws
                 + np.cross(w, np.cross(w, self._s[-1][:3]))
                 + 2 * np.cross(w, self._h[-1])
                 - self._cterm_loom_scale * (np.dot(self._h[-1] + np.cross(w, self._s[-1][:3]), e3)) * self._h[-1]
                 - self._dh_d[-1])

        # Theta matrix and its Frobenius norm
        # Theta = [-c + S*dp - G\(chi*zeta_dot_aug), eye(3)]  (chi_zeta_aug set at the surface above)
        vector = (- c
                  + self._S[-1] @ self._dp[-1]
                  - np.linalg.solve(self._G[-1], chi_zeta_aug))
        Theta = np.hstack([vector.reshape(-1, 1), np.eye(N_DIM)])
        self._theta.append(np.linalg.norm(Theta, ord='fro'))   # scalar ||Theta||_F (logged for continuity)
        # theta used by the control law: per-axis row-norm (decoupled) or the shared scalar (default).
        if self._theta_per_axis:
            theta_ctrl = np.sqrt(vector ** 2 + 1.0)            # theta_i = ||row_i(Theta)|| = sqrt(vec_i^2+1)
        else:
            theta_ctrl = self._theta[-1]

        # Adaptive-gain (translational) update via RK5
        # MATLAB: dkappa/dt = Theta_norm * N * G * |sigma| - N * P * kappa
        # Freeze κ on axes where Singhal outlier containment fired (funnel-breach glitch — the clipped
        # G/σ would drive κ up via the barrier singularity, G→∞ near the funnel edge).
        # NOTE (2026-06-10): a 2nd "sustained-high-θ" κ-freeze trigger was prototyped + DROPPED — it is
        # mis-targeted. At E_Z=0.5 κ_z ratchets up during the MODERATE-θ stretches (median ~3-7, with
        # the freeze OFF) via large G·|σ| at close range — NOT the brief high-θ spikes the trigger
        # chased (θ>50 on only ~19% of frames). No θ threshold (50 or 200) catches it. The lever for
        # the E_Z=0.5 κ-bound is P_z (κ_eq ∝ 1/P leakage). See memory feedback_theta_norm_klt_drift.
        # KAPPA-RATCHET FIX (2026-07-30): freeze kappa (hold last value, skip the ODE
        # integration) while CBF_CORNERS_STALE is True. Root cause found via real hardware
        # flight telemetry the same session: 2 of 4 flights with real closed-loop feedback
        # showed kappa growing 20-24x (0.75->16-18) and a_u peaking at 59-180 (vs single
        # digits in the other 2) during sustained feature loss (cbf_corners none_streak
        # reaching ~400). The kappa-ODE integrates dkappa/dt = Theta*N*G*|sigma| - N*P*kappa
        # -- it has no way to tell "sigma is elevated because of real tracking error" apart
        # from "sigma is elevated because there's no valid measurement to track against, so
        # the feature signal is frozen/stale/extrapolated". On real hardware the latter is
        # common (small marker, narrow FOV -> frequent multi-second coasts) and previously
        # kept ratcheting kappa upward for the ENTIRE coast, so a rare reacquisition then
        # applied an enormous, over-inflated corrective effort. CBF_CORNERS_STALE (added
        # earlier this session, validated both by unit test and live on the Pi) is exactly
        # the "no valid feature data" signal this needs -- gating on it directly parallels
        # the existing FEATURE_IS_STALE-gated behavior elsewhere in this class.
        if len(self._dt) > 0 and not self.CBF_CORNERS_STALE:
            new_kappa = RK5(self._kappaSolver, t, self._kappa[-1],
                            [self._sigma[-1], theta_ctrl], self._dt[-1])
            if hasattr(self, '_contained'):
                new_kappa[self._contained] = self._kappa[-1][self._contained]
            self._kappa.append(np.minimum(new_kappa, self._kappa_max))
        else:
            self._kappa.append(self._kappa[-1])

        # Switching + equivalent control (in barrier-transformed coords)
        # MATLAB:
        #   u_sw + u_eq when summed -> V_a_cd = -G\(u_sw + u_eq)
        sat_sigma = np.clip(self._sigma[-1] / np.diag(self._E), -1.0, 1.0)
        a_v = (- self._Gma @ self._sigma[-1]
               - theta_ctrl * (np.diag(sat_sigma) @ self._G[-1] @ self._kappa[-1])
               + self._G[-1] @ (- c + self._S[-1] @ self._dp[-1])
               - chi_zeta_aug)
        self._a_v.append(a_v)

        a_u = - np.linalg.solve(self._G[-1], a_v)
        if os.environ.get("AU_DECOMP_DBG", "0") == "1":
            # a_u lateral decomposition (bounce-transition diagnosis). a_u = -G^-1 a_v splits into:
            #   reach  = G^-1·Γ·σ                              (proportional reaching)
            #   switch = G^-1·(θ ⊙ (sat(σ)·G·κ))              (adaptive switching)
            #   equiv  = c - S·ṗ                               (equivalent / known-dynamics: loom, ḣ_d, cross)
            #   drift  = G^-1·χζ_aug                           (position-funnel χ_r·ζ_r drift)
            _reach  = np.linalg.solve(self._G[-1], self._Gma @ self._sigma[-1])
            _switch = np.linalg.solve(self._G[-1], theta_ctrl * (np.diag(sat_sigma) @ self._G[-1] @ self._kappa[-1]))
            _equiv  = c - self._S[-1] @ self._dp[-1]
            _drift  = np.linalg.solve(self._G[-1], chi_zeta_aug)
            _res = float(np.linalg.norm((_reach + _switch + _equiv + _drift - a_u)[:2]))
            print("[au] hz=%+.2f sen=%.2f auxy=%6.1f | reach=%6.1f switch=%6.1f equiv=%6.1f drift=%6.1f | kxy=%.2f sigxy=%.2f res=%.2f" % (
                float(self._h[-1][2]), float(np.max(np.abs(self._s_e_n[-1]))), float(np.linalg.norm(a_u[:2])),
                float(np.linalg.norm(_reach[:2])), float(np.linalg.norm(_switch[:2])),
                float(np.linalg.norm(_equiv[:2])), float(np.linalg.norm(_drift[:2])),
                float(np.linalg.norm(self._kappa[-1][:2])), float(np.linalg.norm(self._sigma[-1][:2])), _res))
        # TERMINAL a_u-cap (combined-barrier analog of the V_ds_d commit-cap). In combined mode
        # the lateral demand flows zeta_r->sigma->a_u (NOT V_ds_d), so PLASMC_COMMIT_DSD_MAX is
        # inert — the terminal zeta_r/G^-1 blow-up spikes a_u_xy to ~130 m/s² -> max-tilt -> marker
        # whipped out of FoV. Cap |a_u_xy| once committed (direction preserved) so the spike can't
        # whip out, keeping s_e_n LIVE. PLASMC_COMMIT_AU_MAX=0 -> OFF (acts only when commit_extent>0).
        if self._committed and self._commit_au_max > 0:
            _nau = float(np.linalg.norm(a_u[:2]))
            if _nau > self._commit_au_max:
                a_u[:2] = a_u[:2] * (self._commit_au_max / _nau)
        # TERMINAL-COMMIT LATERAL TAPER (see __init__): once committed, ramp a_u_xy toward the
        # floor so the terminal 1/Z-corrupted lateral flow can't drive a launch — coast + descend.
        if self._commit_lat_taper and self._committed:
            _dtc = self._dt[-1] if (len(self._dt) > 0 and self._dt[-1] > 1e-6) else 0.008
            self._commit_taper_c = max(self._commit_lat_floor,
                                       self._commit_taper_c - _dtc / max(self._commit_ramp_s, _dtc))
            a_u[:2] = a_u[:2] * self._commit_taper_c
        # GLOBAL a_u_xy cap (PLASMC_AU_MAX_XY): bound the lateral accel at ALL altitudes, not just
        # terminal. Diagnosis (2026-06-20): 53% of combined-barrier fly-aways breach AT ALTITUDE
        # (~2.3m) during the off-center approach where a_u_xy hits ~102 (over-aggression overshoots),
        # BEFORE the terminal commit cap fires. Bounding |a_u_xy| globally tames the approach
        # over-aggression while keeping direction (s_e_n live). 0 = OFF.
        if self._au_max_xy > 0:
            _nag = float(np.linalg.norm(a_u[:2]))
            if _nag > self._au_max_xy:
                a_u[:2] = a_u[:2] * (self._au_max_xy / _nag)
        # DESCENT-ONLY a_u_z cap (PLASMC_AU_MAX_Z): bound the DOWNWARD vertical accel at ALL
        # altitudes; climb direction left uncapped. Found 2026-08-03: a_u_z hit -7.4 m/s^2 as a
        # one-shot correction burst when perception reacquired after a coast during which h_z
        # stayed frozen near zero (see project_pi_izeta_kappa_ratchet_fix memory). 0 = OFF.
        if self._au_max_z > 0 and a_u[2] < -self._au_max_z:
            a_u[2] = -self._au_max_z
        # NOTE: the legacy |a_u|>100 abort was removed. PX4 saturates attitude-
        # rate setpoints internally to physical limits (~±220 deg/s); an
        # over-large a_u from a noisy startup PID firing just produces a
        # brief wobble, not a crash. Keeping the controller alive lets the
        # SMC adapt and recover instead of killing the run permanently.
        # _warmup_remaining is retained as a state variable for future use
        # but no longer gates anything here.
        # (The 2026-07-02 target-acceleration FF (PLASMC_TGT_VEL_FF) was REMOVED
        # per user: the "curved-translation lag" it targeted is a self-sustained
        # lateral LIMIT CYCLE, not a lag — and the FF consumed target-pose
        # derivatives forbidden by the manuscript Problem Statement. Oracle-bound
        # results retained in [[project_rover_turning_open]] as an ablation.)
        self._a_u.append(a_u)

    def _yawCtrl(self):
        """Yaw adaptive SMC (MATLAB kappa_a) -> body yaw-rate setpoint u_a."""
        # FULL 2π wrap to [-π, π] (NOT the old factor-of-2 π-fold). Our alpha is a
        # DISAMBIGUATED 2π DIRECTION: img_data._marker_principal_angle uses the
        # [4,3,2,1] corner weights + the 1st-moment centroid displacement to resolve
        # the 180° axis ambiguity ("clean 360°"). The old fold to [-π/2,π/2] re-
        # collapsed that into the π-form, re-creating the 90° saddle (two equilibria
        # 0°/180° with an unstable point at ±π/2): noise flipped e_a across ±π/2 →
        # the yaw-rate command flipped → LIMIT CYCLE. With the 2π wrap the only
        # discontinuity sits at ±180° — which a landing never reaches → no saddle,
        # no limit cycle. (Diverges from MATLAB:483, which used a π-period alpha on
        # asymmetric T_nP3 geometry; the alpha SOURCE stays moment-based — this is
        # NOT the reverted geometric-source swap. 2026-06-07.)
        # ALPHA RATE-LIMIT + SAVGOL filter (default-on): kill the terminal marker-fill corruption.
        # The corruption is a GRADUAL drift (alpha -5°→-22° over ~0.3s as the marker fills the FoV)
        # while genuine yaw is slow (~2°/s) — so (1) RATE-LIMIT the alpha change to YAW_ALPHA_MAX_RATE
        # (caps the ~57°/s corruption drift, passes genuine slow yaw), then (2) SAVGOL-smooth the
        # rate-limited history (removes per-frame jitter). Tracks real yaw, rejects the drift.
        _alpha = float(self._s[-1][3])
        if self._yaw_alpha_filt and self._yaw_alpha_kf and len(self._dt) > 0 and self._dt[-1] > 1e-6:
            # CV-KF path: dynamics + innovation-gated outlier rejection (PLASMC_YAW_ALPHA_KF).
            _alpha = self._yawKFStep(_alpha, self._dt[-1])
        elif self._yaw_alpha_filt and len(self._dt) > 0 and self._dt[-1] > 1e-6:
            if self._alpha_smooth is None:
                self._alpha_smooth = _alpha
            # unwrap raw vs the running smoothed value (alpha is a 2π angle)
            _araw_uw = self._alpha_smooth + np.arctan2(
                np.sin(_alpha - self._alpha_smooth), np.cos(_alpha - self._alpha_smooth))
            _max_step = self._yaw_alpha_max_rate * self._dt[-1]
            _alpha_uw = self._alpha_smooth + float(np.clip(_araw_uw - self._alpha_smooth, -_max_step, _max_step))
            self._alpha_hist.append(_alpha_uw)
            if len(self._alpha_hist) >= 5:
                _w = len(self._alpha_hist) if len(self._alpha_hist) % 2 else len(self._alpha_hist) - 1
                _w = min(_w, self._yaw_alpha_win | 1)
                if _w >= 5:
                    _alpha_uw = float(savgol_filter(np.array(self._alpha_hist), _w, 2)[-1])
            self._alpha_smooth = _alpha_uw
            _alpha = float(np.arctan2(np.sin(_alpha_uw), np.cos(_alpha_uw)))   # re-wrap
        e_a_raw = _alpha - self._s_d[3]
        e_a = np.arctan2(np.sin(e_a_raw), np.cos(e_a_raw))
        self._e_a.append(e_a)

        # Terminal yaw-hold (2026-06-05): the marker ORIENTATION (alpha) becomes
        # unreliable as the board fills the FoV near touchdown — frozen ArUco corners +
        # close-up 2nd-moment jitter. A trace showed the controller acting on a garbage
        # alpha jump -> w_u_z saturated -> the drone SPINS onto a random heading, landing
        # the board 30-45 deg off square (it was aligned ~5 deg until ~0.5 s before
        # touchdown). When the marker is stale, OR the folded alpha-error jumps faster
        # than the body can physically yaw (a perception glitch, not real motion), HOLD:
        # freeze psi_d below + zero w_u[2] in _attCtrl, so the drone lands at its last
        # good aligned heading instead of spinning off-square.
        self._yaw_hold = False
        self._marker_extent.append(self.MARKER_EXTENT_PX)   # logged proximity diagnostic (scale-free marker span)
        # NOTE: a MARKER_EXTENT terminal-hold trigger was tried (freeze the whole command as the
        # marker fills the FoV) but REVERTED 2026-06-08 — its premise (the loom corrupts near
        # ground) was FALSIFIED: the EKF-fused h_z stays reliable (tracks GT loom; reports the
        # runaway, not garbage). The descent runaway is loom UNDER-scaling -> the controller
        # under-arrests, a loom-cal / descent-gain issue — not a perception-death to freeze through.
        if os.environ.get("YAW_TERMINAL_HOLD", "1") == "1":
            if self.FEATURE_IS_STALE:
                self._yaw_hold = True
            elif len(self._e_a) > 1 and len(self._dt) > 0 and self._dt[-1] > 1e-6:
                _de = self._e_a[-1] - self._e_a[-2]
                _de = np.arctan2(np.sin(_de), np.cos(_de))   # 2π wrap (e_a is a full 2π direction; was a stale π-fold)
                if abs(_de) / self._dt[-1] > float(os.environ.get("YAW_HOLD_ALPHA_RATE", "3.0")):
                    self._yaw_hold = True

        # Trapezoidal integral with CONDITIONAL INTEGRATION (proper anti-windup):
        # freeze the yaw integral while the heading-rate command was SATURATED last
        # step (i.e. during a long initial-yaw slew). The integral only needs to
        # accumulate near the target to null steady-state error; letting it wind up
        # over a 100° slew left ie_a pinned (~2.0) so sigma_a = e_a + Omega_a*ie_a
        # stayed positive AT the zero-crossing -> u_a kept commanding yaw -> overshoot
        # past zero (102°->-22°). This REPLACES the old fixed `_ie_a_clamp` band-aid
        # (dropped) and introduces NO new threshold — it reuses the psi_d-rate
        # saturation flag set at the end of this method (prior step). 2026-06-08.
        if len(self._ie_a) == 0:
            self._ie_a.append(0.0)
        elif getattr(self, "_yaw_rate_saturated", False):
            self._ie_a.append(self._ie_a[-1])          # hold — do not integrate while saturated
        else:
            new_int = self._ie_a[-1] + self._dt[-1] * 0.5 * (self._e_a[-1] + self._e_a[-2])
            self._ie_a.append(float(new_int))

        # Sliding surface
        sigma_a = self._e_a[-1] + self._Omega_a * self._ie_a[-1]
        self._sigma_a.append(sigma_a)

        # Adaptive gain via RK5 (scalar)
        if len(self._dt) > 0:
            new_kappa_a = RK5(self._kappa_a_solver, self._t[-1] - self._t0,
                              self._kappa_a[-1], [sigma_a], self._dt[-1])
            self._kappa_a.append(new_kappa_a)
        else:
            self._kappa_a.append(self._kappa_a[-1])

        # Yaw command (MATLAB: u_a = Gamma_a*sigma_a + sat(sigma_a/E_a)*kappa_a + Omega_a*e_a)
        sat_term = float(np.clip(sigma_a / self._E_a, -1.0, 1.0))
        u_a = (self._Gma_a * sigma_a
               + sat_term * float(self._kappa_a[-1])
               + self._Omega_a * e_a)
        # Sign: MATLAB integrates u_a into desired heading psi_d. Body yaw rate
        # to drive heading toward target = same sign as u_a's effect on psi_d
        # but expressed in body frame. NED yaw rate convention matches.
        # LPF the raw u_a before storing — see _tau_ua comment.
        if len(self._u_a) > 0 and len(self._dt) > 0 and self._tau_ua > 0:
            alpha_ua = self._tau_ua / (self._tau_ua + self._dt[-1])
            u_a = alpha_ua * self._u_a[-1] + (1.0 - alpha_ua) * u_a
        self._u_a.append(float(u_a))

        # Anti-windup: psi_d must NOT advance faster than the drone can physically
        # yaw. The body-rate clamp (W_U_MAX) caps the achievable yaw rate; if psi_d
        # integrates the raw u_a (which reaches ~2.3 rad/s) past that, the virtual
        # compass races ahead of the measured yaw -> e_R saturates -> the SO(3)
        # commands max yaw rate -> overshoot and ±180° psi_d windup (the IC yaw
        # divergence: w_u_z saturated 73% of frames, 3 sign-flips; diagnosed
        # 2026-06-04). Cap the psi_d advance rate at YAW_PSID_RATE·W_U_MAX so the
        # inner loop can actually track the setpoint. The `≤ W_U_MAX` ceiling is the
        # load-bearing part (psi_d must not race past the achievable yaw rate); the
        # factor is now 1.0 (full W_U_MAX) — the old 0.7 was over-conservative and
        # throttled convergence from a large initial yaw (2026-06-08, un-throttled
        # for the arbitrary-initial-yaw case; MATLAB has no clamp as it spawns square).
        # The SMC's u_a (logged) is left unclamped so sigma_a/kappa_a keep adapting.
        _w_max = float(os.environ.get("PLASMC_W_U_MAX", "2.0"))   # BAKED 1.0->2.0 2026-06-30 (see body-rate cap note below; same env)
        _psid_rate = float(os.environ.get("PLASMC_YAW_PSID_RATE", "1.0")) * _w_max   # 0.7->1.0: un-throttle psi_d to full W_U_MAX (converge large initial yaw, 2026-06-08)
        _ua_psid = float(np.clip(u_a, -_psid_rate, _psid_rate))
        # Store the rate psi_d actually advances at, for the optional inner-loop
        # yaw-rate feedforward Omega_d=[0;0;u_a] in _attCtrl (PLASMC_YAW_OMEGA_D_FF).
        # Zeroed during yaw-hold below (psi_d frozen -> no reference rate to track).
        self._ua_psid_ff = 0.0 if self._yaw_hold else _ua_psid
        # Heading-rate saturation flag -> next step's CONDITIONAL INTEGRATION
        # (anti-windup on ie_a, above). True while the slew is rate-limited.
        self._yaw_rate_saturated = bool(abs(u_a) > _psid_rate)

        # Virtual-compass integrator (manuscript Eq. `psi d integrator`):
        #   psi_d(t+dt) = wrap[psi_d(t) + u_a * dt]   (u_a rate-limited, see above)
        # No external heading reference enters; psi_d evolves purely from
        # the image-based alpha error via the leakage ASMC.  Frozen during yaw-hold
        # so a garbage terminal alpha can't slew the virtual compass off-square.
        if self._psi_d is not None and len(self._dt) > 0 and not self._yaw_hold:
            self._psi_d = float(
                np.arctan2(np.sin(self._psi_d + _ua_psid * self._dt[-1]),
                           np.cos(self._psi_d + _ua_psid * self._dt[-1]))
            )

    def _vdsKFStep(self, z, dt):
        """Constant-velocity KF on the centroid s_e[:2] → V_ds = velocity state (2,).
        State x=[px,py,vx,vy]; z=position measurement. Predict (CV model) + update."""
        z = np.asarray(z, float)
        if self._vds_x is None:                       # lazy init at the first measurement
            self._vds_x = np.array([z[0], z[1], 0.0, 0.0])
            self._vds_P = np.diag([1e-2, 1e-2, 1.0, 1.0])
            return np.zeros(2)
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        q = self._vds_kf_q
        # CV process-noise (white-accel) Q
        Q = q * np.array([[dt**3/3, 0, dt**2/2, 0], [0, dt**3/3, 0, dt**2/2],
                          [dt**2/2, 0, dt, 0], [0, dt**2/2, 0, dt]], float)
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)
        R = self._vds_kf_r * np.eye(2)
        x = F @ self._vds_x
        P = F @ self._vds_P @ F.T + Q
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x = x + K @ (z - H @ x)
        P = (np.eye(4) - K @ H) @ P
        self._vds_x, self._vds_P = x, P
        return self._vds_kf_scale * x[2:4]   # rescale onto the measured-flow h scale

    def _yawKFStep(self, z_raw, dt):
        """Scalar angle CV-KF on yaw [yaw, yaw_rate] with innovation gating. z_raw = raw alpha
        (wrapped). Returns the wrapped filtered yaw. Genuine slow yaw passes; corruption-drift
        measurements (large innovation) are down-weighted by inflating R via the Mahalanobis gate."""
        if self._yaw_kf_x is None:
            self._yaw_kf_x = np.array([float(z_raw), 0.0]); self._yaw_kf_P = np.diag([1e-2, 1.0])
            return float(z_raw)
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = self._yaw_kf_q * np.array([[dt**3/3, dt**2/2], [dt**2/2, dt]])
        H = np.array([[1.0, 0.0]])
        x = F @ self._yaw_kf_x; P = F @ self._yaw_kf_P @ F.T + Q
        # unwrap the measurement relative to the predicted yaw (angle continuity)
        z = x[0] + np.arctan2(np.sin(float(z_raw) - x[0]), np.cos(float(z_raw) - x[0]))
        S = float(H @ P @ H.T) + self._yaw_kf_r
        nu = z - x[0]                                   # innovation
        d2 = nu * nu / S                                # Mahalanobis distance²
        r_eff = self._yaw_kf_r
        if d2 > self._yaw_kf_gate:                      # GATE: inflate R ∝ d² so the spike is rejected
            r_eff = self._yaw_kf_r * (d2 / self._yaw_kf_gate)
            S = float(H @ P @ H.T) + r_eff
        K = (P @ H.T).flatten() / S
        x = x + K * nu; P = (np.eye(2) - np.outer(K, H.flatten())) @ P
        self._yaw_kf_x, self._yaw_kf_P = x, P
        return float(np.arctan2(np.sin(x[0]), np.cos(x[0])))

    def _cvkfVecRate(self, st, z, dt):
        """Per-axis constant-velocity KF on a vector signal z; returns the RATE state (n,).
        st = {'x':(n,2),'P':(n,2,2)} persisted across calls. Used for dh_d / dw."""
        z = np.asarray(z, float); n = len(z)
        q, r = self._deriv_kf_q, self._deriv_kf_r
        if st["x"] is None:
            st["x"] = np.column_stack([z, np.zeros(n)])
            st["P"] = np.tile(np.diag([1e-2, 1.0]), (n, 1, 1)).astype(float)
            return np.zeros(n)
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = q * np.array([[dt**3/3, dt**2/2], [dt**2/2, dt]])
        H = np.array([[1.0, 0.0]]); out = np.zeros(n)
        for i in range(n):
            x = F @ st["x"][i]; P = F @ st["P"][i] @ F.T + Q
            S = float(H @ P @ H.T) + r; K = (P @ H.T).flatten() / S
            x = x + K * (z[i] - x[0]); P = (np.eye(2) - np.outer(K, H.flatten())) @ P
            st["x"][i], st["P"][i], out[i] = x, P, x[1]
        return out

    def _kappaSolver(self, _, kappa, X):
        sigma = X[0]
        theta = X[1]   # scalar ||Theta||_F, or per-axis 3-vec (PLASMC_THETA_PER_AXIS)
        # theta*(N G |sigma|): scalar broadcasts identically to the old form; a 3-vec multiplies
        # element-wise -> dkappa_i/dt = theta_i N_i G_i |sigma_i| - N_i P_i kappa_i (per-axis bound).
        return theta * (self._N @ self._G[-1] @ np.abs(sigma)) - self._N @ self._P @ kappa

    def _kappa_a_solver(self, _, kappa_a, X):
        sigma_a = X[0]
        # MATLAB: dkappa_a/dt = n_a * |sigma_a| - n_a * p_a * kappa_a
        return self._n_a * abs(sigma_a) - self._n_a * self._p_a * kappa_a

    def _attCtrl(self):
        """Convert V-frame accel a_u + yaw rate u_a -> [body rates; thrust] for PX4."""
        euler = Quaternion(self._quat[-1]).to_angles()  # [roll, pitch, yaw]
        # ---- Measured-attitude yaw source: compass (EKF) or alpha (marker-derived) ----
        # Compass yaw drifts under aggressive maneuvers (EKF heading); the alpha
        # feature s[3] tracks TRUE world yaw drift-free (output-cal: yaw≈0.949·s[3]
        # +0.040, r=0.98). BODY_YAW_SOURCE=alpha rebuilds the measured attitude with
        # EKF roll/pitch (don't drift) + alpha yaw, so the SO(3) error e_R AND the
        # V→inertial accel transform (I_a_raw) become compass-INDEPENDENT. DEFAULT
        # 'alpha' (compass-free) as of 2026-06-04 — the sign-fixed path eliminated the
        # yaw divergence (first non-divergent compass-free landing). Set
        # BODY_YAW_SOURCE=compass for the legacy manuscript behaviour. s[3]->yaw map is
        # env-tunable (K,B). Falls back to compass when no marker (len(_s)==0). Only
        # valid with the moment-2π alpha (s[3]≈world yaw); see moment-yaw-canonical.
        if os.environ.get("BODY_YAW_SOURCE", "alpha") == "alpha" and len(self._s) > 0:
            # NEGATIVE slope: the compass yaw euler[2] we replace is NED, which is
            # ANTI-correlated with alpha (alpha≈+0.949·GT_yaw_ENU, euler[2]_NED≈
            # -GT_yaw_ENU). yaw_alpha must move WITH psi_d as alpha falls (like the
            # compass yaw did) or e_R winds up -> overshoot (diagnosed 2026-06-04: the
            # +0.949 sign made yaw_alpha and psi_d diverge -> ±135° yaw ring-out).
            # PERCEPTION-DEATH GUARD (2026-06-08): when _yaw_hold is active (marker stale,
            # OR alpha jumping faster than the body can yaw = the marker filling the FoV and
            # corrupting alpha), FREEZE the measured yaw at its last-good snapshot instead of
            # rebuilding the measured attitude from the corrupted live s[3]. Before this, the
            # setpoint (psi_d) and outputs (w_u[2], I_a) were held but THIS measured-yaw source
            # still read live alpha -> corrupted e_R / IK roll-pitch -> terminal spin (rep3 158°).
            # Roll/pitch stay live (EKF, drift-free, trustworthy); only the alpha-yaw is held.
            _k = float(os.environ.get("BODY_YAW_ALPHA_K", "-0.949"))
            _b = float(os.environ.get("BODY_YAW_ALPHA_B", "0.0"))
            if getattr(self, "_yaw_hold", False) and getattr(self, "_yaw_c_hold", None) is not None:
                yaw_c = self._yaw_c_hold                              # held last-good alpha yaw
            else:
                _ya = _k * float(self._s[-1][3]) + _b
                yaw_c = float(np.arctan2(np.sin(_ya), np.cos(_ya)))   # wrapped alpha yaw
                self._yaw_c_hold = yaw_c                              # snapshot last-good (perception alive)
            _roll, _pitch = float(euler[0]), float(euler[1])       # EKF roll/pitch (drift-free)
            _cz, _sz = np.cos(yaw_c), np.sin(yaw_c)
            _cp, _sp = np.cos(_pitch), np.sin(_pitch)
            _cr, _sr = np.cos(_roll), np.sin(_roll)
            R = (np.array([[_cz, -_sz, 0], [_sz, _cz, 0], [0, 0, 1.0]])
                 @ np.array([[_cp, 0, _sp], [0, 1, 0], [-_sp, 0, _cp]])
                 @ np.array([[1, 0, 0], [0, _cr, -_sr], [0, _sr, _cr]]))  # Rz(yaw_a)Ry(p)Rx(r)
        else:
            R = Quaternion(self._quat[-1]).to_DCM()
            yaw_c = float(euler[2])
            self._yaw_c_hold = None                                # compass path: release the held snapshot

        # Raw inertial accel (net of gravity).
        # a_u lives in the gravity-LEVELED V frame; MATLAB uses I_R_V = rotz(yaw),
        # Python historically used the full body DCM R (parity item D1) — which
        # MIS-ROTATES the leveled command by the current tilt. During the lateral
        # overshoot the drone tilts 10-35° (IC1 diag 2026-06-19), so the full-DCM
        # transform mis-directs the commanded brake → not delivered → fly-away
        # (lateral wall = commanded-but-not-delivered, NOT perception, NOT a sign bug;
        # feedback_lateral_wall_anti_restoring_au). PLASMC_AU_ROTZ_ONLY=1 uses the
        # MATLAB-correct rotz(yaw_c) only. Default-off pending IC1 A/B.
        if os.environ.get("PLASMC_AU_ROTZ_ONLY", "0") == "1":
            _czA, _szA = np.cos(yaw_c), np.sin(yaw_c)
            R_au = np.array([[_czA, -_szA, 0.0], [_szA, _czA, 0.0], [0.0, 0.0, 1.0]])
        else:
            R_au = R
        I_a_raw = R_au @ self._a_u[-1] - np.array([0.0, 0.0, g])
        self._I_a_raw.append(I_a_raw.copy())

        # ---- Full MATLAB FoV-margin cone (visualControl_IBVS_adaptive.m:443-469) ----
        I_a = I_a_raw.copy()

        # Lateral phase-lead (PLASMC_AU_LEAD, see __init__). y = u + (wp/wz - 1)(u - x),
        # x' = wp(u - x): unity DC gain (no steady-state distortion), +phase in the cycle
        # band, HF gain wp/wz — clipped downstream by the cone clamp below.
        if self._au_lead and len(self._dt) > 0 and self._dt[-1] > 1e-6:
            _dtl = min(float(self._dt[-1]), 0.1)
            _al = 1.0 - np.exp(-self._au_lead_wp * _dtl)
            self._au_lead_x += _al * (I_a_raw[:2] - self._au_lead_x)
            _lead_delta = ((self._au_lead_wp / self._au_lead_wz) - 1.0) \
                * (I_a_raw[:2] - self._au_lead_x)
            if self._au_lead_ratio > 0.0:
                _cap = self._au_lead_ratio * float(np.linalg.norm(I_a_raw[:2]))
                _nd = float(np.linalg.norm(_lead_delta))
                if _nd > _cap > 0.0:
                    _lead_delta *= _cap / _nd
            I_a[:2] = I_a_raw[:2] + _lead_delta

        # 1) Current tilt angle from body-z direction (R[2,2] is body-z's inertial-z component)
        R33 = float(np.clip(R[2, 2], -1.0, 1.0))
        theta_current = np.arccos(R33)

        # 2) Pixel-margin envelope (per axis). With l_fov=0 (default, 2026-06-05) this is CONSTANT
        #    at rho_fov_0 — a fixed visibility limit (see __init__ note). l_fov>0 restores the decay.
        t_elapsed = self._t[-1] - self._t0
        rho_fov_curr = ((self._rho_fov_0 - self._rho_fov_inf)
                        * np.exp(-self._l_fov * t_elapsed)
                        + self._rho_fov_inf)   # (2,)  == rho_fov_0 when l_fov=0

        # 3) Per-corner pixel margins — get latest 4 corners from img_node
        # _feature_pts[-1] is a [prev, curr] frame pair; [-1][1] is current 4 corners.
        # Raw corners are in OpenCV top-left coords; convert to image-centered.
        # CBF small-marker preference (2026-07-25 port, see __init__ comment): prefer
        # PlanarFeatureMap's secondary (= smaller, by construction) slot when it's
        # confidently mapped; fall back to _feature_pts (whatever's live -- normally the
        # big marker until it's gone) otherwise. Both sources go through the SAME
        # freshness discipline: _feature_pts is only trusted via FEATURE_PTS_FRESH; the
        # small-slot read is only trusted via get_slot_confidence.
        cbf_corners = None
        cbf_corners_src = 'none'
        try:
            _pm = getattr(self._img_node, '_planar_map', None)
            _sec = None
            _sec_conf = 0.0
            if _pm is not None and getattr(_pm, 'initialized', False):
                _sec = _pm.secondary_slot_name()
                _sec_conf = _pm.get_slot_confidence(_sec) if _sec is not None else 0.0
            # HYSTERESIS: immediate-off (any sub-threshold frame drops the streak and the
            # switch instantly), N-frame persistence before switching ON -- see __init__
            # comment. Prevents the corner-source (and therefore d_min_fov/theta_cone)
            # from flickering between big-marker-real and small-marker-mapped geometry
            # whenever confidence hovers near the threshold.
            if _sec is not None and _sec_conf >= self._cbf_small_conf_min:
                self._cbf_small_slot_streak += 1
            else:
                self._cbf_small_slot_streak = 0
                self._cbf_small_slot_on = False
            if not self._cbf_small_slot_on and self._cbf_small_slot_streak >= self._cbf_small_slot_on_frames:
                self._cbf_small_slot_on = True
            if self._cbf_small_slot_on and _sec is not None:
                _sp = _pm.get_marker_frame_pts(slot=_sec)
                if _sp is not None and len(_sp) == 4:
                    cbf_corners = np.asarray(_sp, dtype=float)
                    cbf_corners_src = 'small_slot'
                else:
                    self._cbf_small_slot_on = False   # prediction unavailable this frame -- fall back, don't hold a stale "on"
        except (AttributeError, TypeError, ValueError):
            cbf_corners = None
        if cbf_corners is None:
            # FRESHNESS GATE: _feature_pts HOLDS the last real corners indefinitely during
            # a genuine total coast -- reading it unconditionally would feed the cone-clamp
            # a frozen, arbitrarily-stale marker position as if it were live.
            # FEATURE_PTS_FRESH reflects "raw OR rescue succeeded this frame" -- ONLY a
            # genuine total coast (neither raw nor rescue) is unfresh here.
            #
            # BOUNDED COAST-HOLD (ported from PX4_Gazebo/src/controller.py, 2026-07-31):
            # this hardware's raw-decode coast bursts routinely run into the hundreds of
            # frames (project_pi_coast_root_cause, 2026-07-27). With zero bridging, a
            # SINGLE missed frame at the start of any such burst blanked cbf_corners for
            # the ENTIRE burst, feeding directly into CBF_CORNERS_STALE_ABORT and causing
            # 15 of 17 flights in the 2026-07-31 test session to false-abort within
            # seconds. Hold the last-good corner geometry through a BOUNDED number of
            # consecutive coast frames (default 30, PLASMC_CBF_COAST_GRACE) before
            # falling back to no-corners -- low-risk since this only affects the CBF's
            # corner geometry for the visibility barrier, not the flow/position
            # controller directly.
            try:
                if getattr(self._img_node, 'FEATURE_PTS_FRESH', True):
                    fp_list = self._img_node._feature_pts
                    if len(fp_list) > 0:
                        cbf_corners = np.asarray(fp_list[-1][1])   # (4, 2) — (u, v) top-left
                        cbf_corners_src = 'feature_pts'
                        self._cbf_coast_last_good = cbf_corners
                        self._cbf_coast_ctr = 0
                elif (self._cbf_coast_grace_frames > 0
                      and self._cbf_coast_last_good is not None
                      and self._cbf_coast_ctr < self._cbf_coast_grace_frames):
                    self._cbf_coast_ctr += 1
                    cbf_corners = self._cbf_coast_last_good
                    cbf_corners_src = 'coast_hold'
            except (IndexError, AttributeError, TypeError):
                cbf_corners = None

        # CBF-CORNERS STALENESS TRACKING (2026-07-30): investigation this session
        # (see PX4_Gazebo/docs/HANDOFF_cbf_lockout_planarmap_2026-07-30.md, ported
        # here for parity) found real flights where cbf_corners went to None
        # (neither the PlanarFeatureMap small-slot nor the raw _feature_pts
        # source available) for a sustained stretch, silently freezing
        # cbf2_filter's internal state (Phase 2, frozen delta_ref/Lw2_ref/
        # cr_prev/d) for 30+ seconds -- while TARGET_IS_VISIBLE/FEATURE_IS_STALE
        # (the signals hardware_landing.py's feature_fresh actually watches)
        # kept reporting fine, because they reflect a DIFFERENT signal than
        # what gates cbf_corners. The CBF degraded into a near-zero-authority
        # state with NOTHING in the app loop noticing or falling back to the
        # (already-validated) MARKER_LOSS_GRACE open-loop fallback. This
        # counter + property exposes that staleness so hardware_landing.py can
        # watch it directly, the same way it already watches FEATURE_IS_STALE.
        if cbf_corners is None:
            self._cbf_corners_none_streak = getattr(self, "_cbf_corners_none_streak", 0) + 1
        else:
            self._cbf_corners_none_streak = 0
        if os.environ.get("PLANAR_MAP_DBG", "0") == "1":
            self._cbf_corners_dbg_ctr = getattr(self, "_cbf_corners_dbg_ctr", 0) + 1
            if self._cbf_corners_dbg_ctr % 15 == 0:
                _fresh = getattr(self._img_node, "FEATURE_PTS_FRESH", None)
                print(f"[cbf_corners] src={cbf_corners_src} FEATURE_PTS_FRESH={_fresh} "
                      f"corners_is_none={cbf_corners is None} "
                      f"none_streak={self._cbf_corners_none_streak}", flush=True)

        d_min_fov = 0.0
        self._cbf_overflow = False
        self._cbf_drift_off = False
        self._cbf_drift_axis = None
        if cbf_corners is not None:
            try:
                cx, cy = self._img_node.center
                u_centered = cbf_corners[:, 0] - cx
                v_centered = cbf_corners[:, 1] - cy
                d_corner_x = rho_fov_curr[0] - np.abs(u_centered)   # (4,)
                d_corner_y = rho_fov_curr[1] - np.abs(v_centered)   # (4,)
                d_min_fov = max(float(np.min(np.concatenate([d_corner_x, d_corner_y]))), 0.0)

                # OVERFLOW vs DRIFT-OFF: classify off the SAME per-corner margin
                # d_min_fov is built from. OVERFLOW = corners breach on OPPOSITE sides of
                # an axis (spanning -- still over target, benign, marks the BIG marker
                # ready for handover). DRIFT-OFF = breach on ONE side only (target
                # visibility genuinely failing -- the CBF's own job to prevent).
                bx_neg = bool(np.any(u_centered < -rho_fov_curr[0]))
                bx_pos = bool(np.any(u_centered >  rho_fov_curr[0]))
                by_neg = bool(np.any(v_centered < -rho_fov_curr[1]))
                by_pos = bool(np.any(v_centered >  rho_fov_curr[1]))
                span = (bx_neg and bx_pos) or (by_neg and by_pos)
                leaving = bx_neg or bx_pos or by_neg or by_pos
                self._cbf_overflow = bool(leaving and span)
                self._cbf_drift_off = bool(leaving and not span)
                if self._cbf_drift_off:
                    # worst (most negative) per-axis margin picks the pull-back axis/sign
                    _cands = []
                    if bx_neg and not bx_pos: _cands.append((0, -1, float(d_corner_x.min())))
                    if bx_pos and not bx_neg: _cands.append((0, +1, float(d_corner_x.min())))
                    if by_neg and not by_pos: _cands.append((1, -1, float(d_corner_y.min())))
                    if by_pos and not by_neg: _cands.append((1, +1, float(d_corner_y.min())))
                    if _cands:
                        self._cbf_drift_axis = min(_cands, key=lambda c: c[2])[:2]
            except (IndexError, ValueError, TypeError):
                d_min_fov = 0.0

        # 4) Cone angle = current tilt + tilt-headroom-before-the-marker-exits, capped.
        focal_px = float(np.atleast_1d(self._img_node.focal)[0])
        # Visibility tilt-cone headroom = current tilt + how far we can still tilt
        # before the nearest marker corner exits the FoV envelope, capped at theta_cap.
        # This is the cbf2 Phase-2 fallback cone (the exact camera-frame theta-QP below
        # refines it when corners are available).
        theta_cone = float(min(theta_current + np.arctan(d_min_fov / focal_px),
                               self._theta_cap))
        # θ_cone floor — RESTORED 2026-06-03 (removed by the 14:20 refactor; second
        # refactor regression after DH_D_MAX). The d_min collapse logic assumes
        # tilt moves the marker OUT of the image, but tilting toward the marker
        # re-centers it — near touchdown / during overshoot d_min→0 collapses
        # θ_cone to the current tilt and clamps exactly the recovery action
        # (cone-clamp duty 43.8% vs 4.9% with the floor; LateralRestore c1 vs b13).
        # The validated 28%-SP config ran with floor=60 (=θ_cap, disables the
        # d_min term); it is the default. Set PLASMC_THETA_FLOOR_DEG=0 for the
        # legacy collapsing cone, or 15-30 for a softened intermediate clamp.
        theta_cone = float(max(theta_cone, min(self._theta_floor, self._theta_cap)))

        # 5) Apply cone to inertial accel (NED; z=down, gravity subtracted).
        # MATLAB-equivalent safety: I_a represents required thrust acceleration
        # (thrust force / m). For sane upright drone with thrust opposing
        # gravity, I_a[2] should be NEGATIVE (thrust accel up in NED). If the
        # SMC ever commands I_a[2] >= 0, that asks for non-upward thrust →
        # drone would have to flip. Force a moderate negative value (~-3 m/s²
        # of thrust accel, i.e. mild lift but well below hover) to keep the
        # drone upright; the controller can still descend by reducing thrust
        # below gravity.
        if I_a[2] >= 0:
            I_a[2] = -3.0
        # Low-pass the DESIRED accel BEFORE the CBF, so the QP re-imposes the hard FoV
        # bound on the FILTERED input (clean attitude, bound NOT smeared — filtering the
        # CBF OUTPUT would smear the bound). Ported from the MATLAB result
        # (UBUNTU_HANDOFF.md §2): in MATLAB, pre-CBF filtering recovered a soft touchdown.
        # DEFAULT-OFF (REVERTED 2026-06-15): the MATLAB benefit did NOT reproduce in SITL.
        # The s_e_n<=p_s metric showed CBF_LPF_BEFORE=1 makes it WORSE — the added attitude
        # lag breaches the outer funnel EARLY at altitude (2-4 m) on the centered IC1
        # descent (all 5 reps) vs LPF-after holding to the terminal ~1 m (2/5 clean); also a
        # lateral wash at IC2 + catastrophic with KP-up. =1 to re-enable (handoff TASK A).
        _lpf_before = os.environ.get("CBF_LPF_BEFORE", "0") == "1"
        if _lpf_before and len(self._I_a) > 0 and len(self._dt) > 0:
            _a = self._tau_ia / (self._tau_ia + self._dt[-1])
            I_a = _a * self._I_a[-1] + (1.0 - _a) * I_a
        # Visibility constraint = exact camera-frame theta-QP (docs/CBF_visibility.pdf —
        # the literal QP). Extracted verbatim into cbf_visibility.cbf2_filter so the
        # offline validators (tools/validate_cbf.py, tools/replay_cbf.py) run the EXACT
        # live code path. The barrier, two-phase δ, and Phase-2 fallback all live there;
        # this site only marshals the controller state into pure args.
        self._theta_safe = None        # Fix B: cbf2 Phase-1 safe lean vector for direct->rd3; None => accel path
        # Reuse the SAME cbf_corners computed above for d_min_fov (small-slot-preferred,
        # freshness-gated) rather than re-reading raw _feature_pts here -- keeps the exact
        # QP and the Phase-2 fallback cone consistent on which corner source they see.
        corners = cbf_corners
        dt_last = self._dt[-1] if len(self._dt) > 0 else None
        w_rp = np.asarray(self._w[-1][:2], float) if len(self._w) > 0 else np.zeros(2)
        I_a, theta_cone, _cbf_ok, self._theta_safe = cbf2_filter(
            I_a, R, R33, yaw_c, corners,
            self._img_node.center, self._img_node.focal,
            self._p_10, theta_cone,
            dt_last, w_rp, self._cbf_state)
        # Deliverable-tilt cap (theta_cap saturation) — applied HERE, not in the CBF:
        # it is a thrust-deliverability bound, not a visibility constraint. The CBF
        # returns the un-capped safe lean th_safe (and I_a[:2]=a_z·Rz@th_safe), so a
        # single scale clips both consistently (|I_a[:2]| = a_z·|th_safe|).
        if self._theta_safe is not None:
            _tn = float(np.linalg.norm(self._theta_safe))
            if _tn > self._theta_cap:
                _scl = self._theta_cap / _tn
                self._theta_safe = self._theta_safe * _scl
                I_a[:2] = I_a[:2] * _scl
            theta_cone = float(np.linalg.norm(self._theta_safe))   # log the capped commanded tilt
        I_a[2] = max(I_a[2], -50.0)

        # log FoV diagnostics
        self._rho_fov_log.append(rho_fov_curr.copy())
        self._d_min_fov_log.append(d_min_fov)
        self._theta_cone_log.append(theta_cone)
        self._theta_current_log.append(theta_current)

        # ---- LPF (MATLAB tau_ia = 0.08 s) ----
        if _lpf_before:
            # already filtered pre-CBF; store the CBF-constrained result as-is
            # (filtering again here would smear the hard FoV bound the QP just set).
            self._I_a.append(I_a.copy())
        elif len(self._I_a) == 0:
            self._I_a.append(I_a.copy())
        else:
            alpha = self._tau_ia / (self._tau_ia + self._dt[-1])
            self._I_a.append(alpha * self._I_a[-1] + (1.0 - alpha) * I_a)

        # Terminal stabilization (2026-06-05): on perception-stale (yaw-hold active near
        # touchdown) the drone TILTS 14-32 deg chasing the frozen stale optic flow (all of
        # the lateral drift is gained in this blind phase) AND the SMC winds up the thrust
        # (free-fall). Override the desired accel to LEVEL + a gentle descent so the drone
        # holds itself stable through the blind final ~0.5 m: a vertical I_a -> Gram-Schmidt
        # R_d is level (e_R[:2]->0, no horizontal force => lateral hold) and B_T = mass*eps
        # is a gentle descent. Yaw is already held by w_u[2]=0. Both R_d and B_T read
        # self._I_a[-1], so overriding it in place drives all three. Scale-free: only the
        # drone's own g + a commanded descent accel (no target altitude/Z).
        # Terminal stabilization — HOLD LAST-GOOD (2026-06-05). When perception goes stale near
        # touchdown, FREEZE the whole control output I_a at its last trustworthy value (snapshot at
        # the first stale frame, hold through the blind phase) instead of letting it wind up on the
        # stale optic flow. HOLDING (not forcing) keeps R_d ~= the actual attitude -> small e_R -> no
        # instability — forcing a LEVEL R_d here DIVERGED (frozen-yaw mismatch corrupts e_R[0:2] ->
        # w_u saturates -> drone pitches further off + falls; n=2 rel_vel 0.5-1.0 vs gate-only 0.49).
        # And it holds the last-good thrust so the drone keeps its pre-death law-descent (~h_d*Z)
        # instead of free-falling. Both R_d and B_T read self._I_a[-1], so holding it drives all
        # axes; yaw is already held by w_u[2]=0. Scale-free: freezes an image-derived command only.
        if getattr(self, "_yaw_hold", False) and os.environ.get("TERMINAL_STABILIZE", "0") == "1":
            if getattr(self, "_Ia_hold", None) is None:
                self._Ia_hold = self._I_a[-1].copy()    # snapshot last-good at perception-death onset
            self._I_a[-1] = self._Ia_hold               # hold it through the blind phase
        else:
            self._Ia_hold = None                        # perception alive -> release the hold

        # ---- Inverse kinematics: desired roll/pitch from I_a (use current yaw) ----
        I_a_use = self._I_a[-1]
        # yaw_c set above per BODY_YAW_SOURCE (compass euler[2] or alpha s[3])
        cy, sy = np.cos(yaw_c), np.sin(yaw_c)

        if abs(cy * I_a_use[0] + sy * I_a_use[1]) < 1e-6:
            theta_d = 0.0
        else:
            theta_d = np.arctan2(-cy * I_a_use[0] - sy * I_a_use[1], -I_a_use[2])

        if abs(sy * I_a_use[0] - cy * I_a_use[1]) < 1e-6:
            phi_d = 0.0
        else:
            phi_d = np.arctan2(-sy * I_a_use[0] + cy * I_a_use[1],
                               -I_a_use[2] / np.cos(theta_d))

        # Clamp away from singularity (phi_d, theta_d used only for logging
        # and the inverse-kinematics decomposition; R_d below is built from
        # I_a directly via the manuscript recipe).
        ang_lim = np.deg2rad(85.0)
        phi_d = float(np.clip(phi_d, -ang_lim, ang_lim))
        theta_d = float(np.clip(theta_d, -ang_lim, ang_lim))

        # ===== Manuscript inner loop (control_formulation.tex:219-251) =====
        # 1. psi_d state (integrated in _yawCtrl); lazy-init on first call
        #    from current body yaw (MATLAB line 127: psi_d = yaw_init).
        if self._psi_d is None:
            self._psi_d = float(yaw_c)

        # 2. R_d construction (Eq. `R_d construction`):
        #      rd3 = -I_a / ||I_a||       (desired body-z opposes net force, NED)
        #      a_h = [cos psi_d, sin psi_d, 0]  (heading vector)
        #      rd2 = (rd3 × a_h) / ||rd3 × a_h||  (Gram-Schmidt)
        #      rd1 = rd2 × rd3
        f_mag = float(np.linalg.norm(I_a_use))
        if f_mag < 1e-6:
            R_d = np.eye(3)
        else:
            # Fix B (2026-06-14): when cbf2 produced a safe lean vector th_safe,
            # build the desired body-z DIRECTION from it directly
            #   rd3 = [-Rz(yaw)@th_safe, 1] / norm
            # instead of from the LPF'd accel. a_z cancels in -I_a/||I_a|| so the
            # direction is identical absent filtering; this removes the LPF smear
            # (tau_ia=0.08, alpha~0.83) of the CBF's HARD visibility bound, so the
            # deliverable-tilt guarantee lands exactly on the commanded attitude
            # (the round-trip tilt->accel->LPF->tilt no longer leaks). Vertical
            # thrust magnitude (B_T) still comes from the filtered/loom I_a[2].
            # Env CBF_RD3_DIRECT=0 reverts to the accel path for A/B.
            ts = self._theta_safe
            if ts is not None and os.environ.get("CBF_RD3_DIRECT", "1") == "1":
                a_xy_dir = np.array([cy * ts[0] - sy * ts[1],
                                     sy * ts[0] + cy * ts[1]])      # Rz(yaw)@th_safe
                rd3 = np.array([-a_xy_dir[0], -a_xy_dir[1], 1.0])
                rd3 = rd3 / np.linalg.norm(rd3)
            else:
                rd3 = -I_a_use / f_mag
            a_h = np.array([np.cos(self._psi_d), np.sin(self._psi_d), 0.0])
            rd2_raw = np.cross(rd3, a_h)
            n2 = float(np.linalg.norm(rd2_raw))
            if n2 < 1e-6:
                # degeneracy guard (manuscript: rd2 = inertial East)
                rd2 = np.array([0.0, 1.0, 0.0])
            else:
                rd2 = rd2_raw / n2
            rd1 = np.cross(rd2, rd3)
            R_d = np.column_stack([rd1, rd2, rd3])

        # 3. SO(3) attitude error (Eq. `so3 errors`):
        #      e_R = 0.5 * vee(R_d^T R - R^T R_d)
        eR_mat = 0.5 * (R_d.T @ R - R.T @ R_d)
        e_R = np.array([eR_mat[2, 1], eR_mat[0, 2], eR_mat[1, 0]])

        # 4. Body-rate setpoint: w_u = -K_R · e_R.
        # Proportional part of the manuscript's full SO(3) torque law
        #     tau = -k_R · e_R  -  k_Omega · e_Omega  +  omega × J · omega
        # The damping and gyroscopic feedforward terms are absorbed into
        # PX4's onboard rate controller (since we ship body rates here,
        # not torques, via MAVSDK set_attitude_rate).
        w_u = -self._K_R @ e_R

        # Optional inner-loop YAW-RATE FEEDFORWARD Omega_d = [0;0;u_a]
        # (PLASMC_YAW_OMEGA_D_FF=1, default OFF): add the rate psi_d is actually
        # advancing at (the rate-limited ASMC output) to the yaw body-rate
        # command, so the inner loop TRACKS the moving psi_d instead of lagging
        # it through -K_R·e_R alone. MATLAB tried the same FF in so3_tracker:
        # helped CONSTANT-rate yaw (Circular e_a 180->53 deg) but broke the
        # oscillating Sinusoidal reference (it feeds back the ASMC's own lagged
        # output -> circular for fast-varying u_a) -> use for constant-rate
        # rotating targets (the rover), keep OFF elsewhere.
        # [[project_yaw_observability_campaign]] / [[project_rover_turning_open]]
        if os.environ.get("PLASMC_YAW_OMEGA_D_FF", "0") == "1":
            w_u = w_u.copy()
            w_u[2] += getattr(self, "_ua_psid_ff", 0.0)

        # Hard clamp on body-rate command magnitude. LK optical flow has a
        # ~15 px tracking window; at our 540 px focal length and 60 Hz
        # image rate, a body rate of ~1.7 rad/s = 100°/s already pushes
        # corner motion to the LK limit. Clamping at 1.0 rad/s (= 57°/s)
        # gives a safety margin and prevents transient attitude-error
        # spikes from breaking optical-flow tracking (2026-05-21 finding).
        # Env-overridable.
        # ⚠ 2026-06-29 TUNING-PHASE DISABLE (default 1.0 -> 1000 = OFF): the W_U_MAX body-rate cap
        # was MASKING the terminal limit cycle — it bounded the cycle AMPLITUDE (GT-FB uncap test:
        # removing it grew the cycle 1.35->1.62 + moved it to ~14Hz bang-bang chatter, but ALSO
        # tightened precision xy 0.087->0.055 4/4). Per [[feedback_clamps_during_tuning]] (disable the
        # band-aid to EXPOSE the under-tuned law, then fix at the control level). The exposed 14Hz
        # chatter = un-damped high-bandwidth SMC switching once the 1/Z forcing ejects sigma from the
        # boundary layer. MUST re-engage (or replace with super-twisting / the descent governor that
        # removes the 1/Z forcing) BEFORE perception-ON — uncapped rates break LK tracking (the original
        # 2026-05-21 reason for the cap). Yaw psi_d ceiling (line ~1781) keeps its own 1.0 default.
        # RE-ENGAGED 2026-06-30 (1000->1.0): the cap was disabled to expose the bang-bang, which was a
        # GT-FB artifact of the non-physical Z_REG=0.01; with Z_REG=0.2 (gear floor) sigma stays in the
        # boundary layer (no bang-bang), so the cap is INERT and safe to restore (LK perception needs it).
        # BAKED 1.0->2.0 (2026-06-30, user): on P2INF=1.5 the 1.0 cap was CLAMPING the terminal body-rate
        # ~24% on the not-soft reps, and the clamp DISCONTINUITY (hitting the rate limit) SEEDS/amplifies the
        # terminal lateral limit cycle (clamp -> bang-bang -> larger cmd -> more clamp). At 2.0 the cmd stays
        # below the cap (IC5r2 SP rep: terminal |w_u| max 0.28 << 2.0, 0% clamped) so the discontinuity never
        # fires -> the cycle isn't seeded -> smooth settle -> SP (IC5r2 rel 0.545->0.0075). NB the cmd is
        # LOWER at 2.0, not higher -- the cap was a cycle DRIVER, not a throttled brake. 2.0 still bounds the
        # worst chatter (unlike the 1000-uncap, which grew the cycle in the OLD Z_REG=0.01 regime). LK-safe.
        w_max = float(os.environ.get("PLASMC_W_U_MAX", "2.0"))
        w_u = np.clip(w_u, -w_max, w_max)

        # Terminal yaw-hold (see _yawCtrl): when the marker orientation is unreliable
        # near touchdown, don't drive yaw on the garbage alpha — zero the yaw-rate
        # command so the drone holds its last aligned heading (else it spins off-square).
        if getattr(self, "_yaw_hold", False):
            w_u[2] = 0.0

        # Diagnostics: log desired-attitude decomposition + SO(3) error
        self._euler_d.append(np.array([phi_d, theta_d, self._psi_d]))
        self._e_R_log.append(e_R.copy())

        self._w_u.append(w_u)

        # Thrust scalar (Newtons) for landing_test.convert_2_sys_cmd, which
        # expects B_T as "thrust DEFICIT below hover": thrust_norm = 0.738 -
        # B_T/45. So B_T = 0 at hover, B_T > 0 → less thrust (descend),
        # B_T < 0 → more thrust (climb).
        # Our I_a[2] is in NED-with-gravity-subtracted (so I_a[2] = -g at hover,
        # not 0). Need to add g back to align: B_T = mass·(I_a[2] + g)/cos·cos.
        # Without the +g, B_T = -20.7 N at hover → thrust_norm clips to 1.0 →
        # full throttle → drone climbs instead of hovering.
        self._B_T.append(mass * (self._I_a[-1][2] + g)
                         / max(np.cos(euler[0]), 1e-6)
                         / max(np.cos(euler[1]), 1e-6))

        self._u.append(np.concatenate((self._w_u[-1], [self._B_T[-1]])))

    # ---------------- Public API ----------------
    def startController(self, warmup_steps=100):
        self._t0 = self._time.perf_counter()
        self._CONTROLLER_READY = True
        # During warmup, suppress the |a_u|>100 abort so the PID/SMC deques
        # can fill without triggering the safety. Caller (landing_test) should
        # NOT use the controller output during this phase — keep sending hover
        # setpoints instead. Set to 0 to disable warmup.
        self._warmup_remaining = warmup_steps

    def checkTargetVisibility(self):
        # Backward-compat: outer barrier removed, so visibility is delegated to img_node
        return bool(self._img_node.FEATURE_IS_VISIBLE)

    def getControlInput(self):
        return self._u[-1] if len(self._u) > 0 else np.zeros(N_DIM + 1)

    def getParams(self):
        return {
            "Des Img Feature Param": self._s_d,
            # Outer PID
            "p_10": self._p_10,
            "K_rp": self._K_rp,
            "K_ri": self._K_ri,
            "K_rd": self._K_rd,
            # Middle-loop envelope
            "gamma": self._gamma,
            "p_0": self._p_0,
            "p_inf": self._p_inf,
            # Middle-loop SMC
            "Omega": self._Omega,
            "Gamma": self._Gma,
            "E": self._E,
            # Adaptive gains
            "N": self._N,
            "P": self._P,
            "kappa_0": self._kappa_0,
            "kappa_max": self._kappa_max,
            # Yaw SMC
            "Omega_a": self._Omega_a,
            "Gamma_a": self._Gma_a,
            "n_a": self._n_a,
            "p_a": self._p_a,
            "kappa_a_0": self._kappa_a_0,
            "E_a": self._E_a,
            # FoV / LPF
            "theta_cap_deg": np.rad2deg(self._theta_cap),
            "theta_floor_deg": np.rad2deg(self._theta_floor),
            "rho_fov_0": self._rho_fov_0,
            "rho_fov_inf": self._rho_fov_inf,
            "l_fov": self._l_fov,
            "tau_ia": self._tau_ia,
            "tau_ua": self._tau_ua,
            # Outer SEN_FUNNEL
            "sen_funnel": self._sen_funnel,
            "gamma_s": self._gamma_s,
            "p_s_0": self._p_s_0,
            "p_s_inf": self._p_s_inf,
            # Outer PID LPF / descent clamp
            "tau_ds": self._tau_ds,
            "DH_D_MAX": self._DH_D_MAX,
            # SO(3) inner-loop gain
            "K_R": self._K_R,
        }

    def getLogData(self):
        """Full-flight log: archived pre-re-init segments + the live segment, merged
        per key. The time axis 't' stays monotonic across segments (perf_counter),
        so a re-init shows up as continuous data — locate re-inits via kappa
        resetting to kappa_0 or the p_2 funnel re-widening."""
        cur = self._buildLogDict()
        if not self._log_segments:
            return cur
        merged = {}
        for k in cur:
            vals = []
            for seg in self._log_segments:
                vals.extend(seg.get(k, []))
            vals.extend(cur[k])
            merged[k] = vals
        return merged

    def _buildLogDict(self):
        return {
            "t": self._t,
            "w_i(t)": self._w_i,
            "w(t)": self._w,
            "h(t)": self._h,
            "h_d(t)": self._h_d,
            "h_d_noS(t)": self._h_d_noS,        # DIAG: transport+descent (h_d minus rate term)
            "s_dot_meas(t)": self._s_dot_meas,  # DIAG: measured centroid rate
            "hd_rate(t)": self._hd_rate_log,    # DIAG: the _hd_rate term (funnel-ref or s_dot)
            "MARKER_EXTENT_PX(t)": self._marker_extent,
            "dh_d(t)": self._dh_d,
            "s(t)": self._s,
            "s_e(t)": self._s_e,
            "s_e_n(t)": self._s_e_n,
            "is_e_n(t)": self._is_e_n,
            "ds_d(t)": self._ds_d,
            # Outer-loop position funnel envelope (SEN_FUNNEL); residency r=s_e_n/p_s
            "p_s(t)": self._p_s,
            "dp_s(t)": self._dp_s,
            "p_r(t)": self._p_r,            # combined-barrier position funnel (s_e_n vs p_r)
            "zeta_r(t)": self._zeta_r,      # combined-barrier position barrier
            # Middle-loop barrier
            "p(t)": self._p,
            "dp(t)": self._dp,
            "S(t)": self._S,
            "zeta(t)": self._zeta,
            "izeta(t)": self._izeta,
            "G(t)": self._G,
            "theta(t)": self._theta,
            "sigma(t)": self._sigma,
            "kappa(t)": self._kappa,
            # Yaw SMC
            "e_a(t)": self._e_a,
            "ie_a(t)": self._ie_a,
            "sigma_a(t)": self._sigma_a,
            "kappa_a(t)": self._kappa_a,
            "u_a(t)": self._u_a,
            # Attitude / output
            "a_v(t)": self._a_v,
            "a_u(t)": self._a_u,
            "I_a_raw(t)": self._I_a_raw,
            "I_a(t)": self._I_a,
            "w_u(t)": self._w_u,
            "B_T(t)": self._B_T,
            "EA_d(t)": self._euler_d,
            "e_R(t)": self._e_R_log,
            # FoV-margin cone diagnostics
            "rho_fov(t)": self._rho_fov_log,
            "d_min_fov(t)": self._d_min_fov_log,
            "theta_cone(t)": self._theta_cone_log,
            "theta_current(t)": self._theta_current_log,
        }

    def getImgData(self):
        return self._img_node.getLogData()

    def getImgParams(self):
        return self._img_node.getParams()

    def enableRecording(self):
        self._img_node.RECORD = True
