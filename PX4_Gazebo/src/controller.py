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
from threading import Thread
from collections import deque

from numerical_methods import RK5, smooth4
import img_data as ID
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
    def __init__(self, ref_rad_opt_flow, des_img_feature, time_keeper=time, controller=None, record='n'):
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
        self._gamma = np.diag(pa("XI2",   0.6, 0.6, 0.8))   # XI2_z 0.6->0.8 (2026-06-13 user bake, Ez5_combo) — faster z funnel contraction
        self._p_0   =         pa("P20",   25.0, 25.0, 10.0)  # P20_z 4->10 (2026-06-13 user bake, Ez5_combo) — wider initial z funnel
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
        self._N       = np.diag(pa("N",      0.02, 0.02, 0.1))   # N_z 0.05->0.1 (2026-06-13 soft-config bake): faster κ_z adaptation -> z braking authority arrives in time (descent is only ~2s; κ_z is adaptation-rate-limited)
        self._P       = np.diag(pa("P",      1.5, 1.5, 5.0))     # P_xy 5->1.5 (2026-06-12 Combo bake): MATLAB parity; paired with KP=5 the lateral kappa stays bounded without the over-gained P=5
        self._kappa_0 =         pa("KAPPA0", 0.5, 0.5, 1.0)     # KAPPA0_xy 0.125->0.5 (gain-chain crossing brake); KAPPA0_Z 0.25->1.0 (2026-06-13 soft-config bake): the BOOTSTRAP value — z braking authority from t=0 -> soft touchdown (vel 4.4->1.3-1.8 m/s) AND prevents the E_z=0.1 κ_z ratchet
        self._kappa_max = pa("KAPPA_MAX", 1e6, 1e6, 3.0)                # KAPPA_MAX_Z=3.0 (REVERTED from 10.0, 2026-06-13): the IC2-5 gate CONFIRMED 10.0 is net-negative — κ_z ran to 10 in the drift/hard reps (IC3_rep4 11.5 m/s, IC4) where 3.0 would have held it (more violent), while clean soft reps sit at κ_z~1 (cap irrelevant). The 3.0 backstop is load-bearing in bad reps, inert in good ones.
        self._dw_max    = float(os.environ.get("PLASMC_DW_MAX", "30.0"))   # physical clamp on |dw| (rad/s²) for the c-term feedforward

        # ════ COMBINED-BARRIER sliding surface (manuscript combined surface; default OFF) ════
        # Aligned to the canonical MATLAB realization (visualControl_IBVS_adaptive.m, a152479).
        # Replaces the SEN back-map: the position barrier zeta_r enters sigma DIRECTLY
        # (lateral sigma_k = zeta_h_k + chi_r*zeta_r_k), so there is no G_s^-1->0 demand
        # starvation (the lateral-wall / IC5 deficit). h_d uses the MEASURED centroid rate
        # s_dot (no back-mapped ds_d); s_ddot is dropped from dh_d (kappa absorbs it as d_h).
        # zeta_r is built on r_bar_e = s_e/p_10 (= s_e_n, FoV-normalized) with its OWN funnel
        # p_r (FoV-consistent floor p_r_inf>=1 — proof Standing Condition 1; precision comes
        # from p_2/chi_r, NOT from tightening p_r). MATLAB-validated 25/25 SP + 75/75 noisy.
        self._combined_barrier = os.environ.get("PLASMC_COMBINED_BARRIER", "0") == "1"
        self._chi_r   = np.array([float(os.environ.get("PLASMC_CHI_R_X", "0.85")),
                                  float(os.environ.get("PLASMC_CHI_R_Y", "0.85"))])   # surface gain (manifold |zeta_r|~|zeta_h|/chi_r); 0.85 = MATLAB max-margin
        self._p_r_0   = np.array([float(os.environ.get("PLASMC_PR0_X", "1.2")),
                                  float(os.environ.get("PLASMC_PR0_Y", "1.2"))])      # position-funnel initial half-width (FoV units)
        self._p_r_inf = np.array([float(os.environ.get("PLASMC_PRINF_X", "1.0")),
                                  float(os.environ.get("PLASMC_PRINF_Y", "1.0"))])    # terminal floor — FoV-consistent (>=1 keeps the CBF->funnel transfer exact)
        self._xi_r    = np.diag([float(os.environ.get("PLASMC_XIR_X", "0.10")),
                                 float(os.environ.get("PLASMC_XIR_Y", "0.10"))])      # position-funnel contraction rate
        # Combined mode uses a tighter lateral optic-flow-funnel floor (chase-lag terminal
        # velocity; proof manifold). Apply only in combined mode, only if not explicitly set.
        if self._combined_barrier:
            if "PLASMC_P2INF_X" not in os.environ: self._p_inf[0] = 0.5
            if "PLASMC_P2INF_Y" not in os.environ: self._p_inf[1] = 0.5

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
        # covered by the per-axis checker above (e.g. FUNNEL_MODE, THETA_FLOOR_DEG).
        _fov_vars = [
            ("FUNNEL_MODE",          "cbf2"),
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
            ("PLASMC_CH_CLEAN",      "0"),
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
        self._Omega_a   = float(os.environ.get("PLASMC_YAW_OMEGA",  "0.5"))
        self._Gma_a     = float(os.environ.get("PLASMC_YAW_GAMMA",  "0.5"))
        self._n_a       = float(os.environ.get("PLASMC_YAW_N",      "1.0"))
        self._p_a       = float(os.environ.get("PLASMC_YAW_P",      "2.0"))
        self._kappa_a_0 = float(os.environ.get("PLASMC_YAW_KAPPA0", "2.0"))
        self._E_a       = float(os.environ.get("PLASMC_YAW_E",      "3.0"))

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
        self._funnel_mode = os.environ.get("FUNNEL_MODE", "cbf2")
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
        # LEVER 2 (flow ceiling + smoothing): blend the accurate DETECTED-centroid rate
        # d(s[:2])/dt into the lateral flow h[:2]. The LK flow saturates ~1 rad/s AND
        # carries lstsq garbage spikes; the centroid rate has no LK ceiling and no lstsq
        # garbage (clean detected position -> clean derivative) -> raises the ceiling AND
        # smooths in one move (both s and h are V-frame, so d(s)/dt IS the translational
        # flow). FLOW_CENTROID_RATE in [0,1]: 0 = pure LK (default OFF), 1 = pure centroid.
        self._FLOW_CENTROID_RATE = float(os.environ.get("FLOW_CENTROID_RATE", "0.0"))
        # Consistent c_h kinematics correction (manuscript §II): clean c-term + drop w×s from h_d.
        # Removes the noisy V_w/V_dw cross-products + de-dominates the rotation-FF in h_d (the
        # a_u-outward overshoot cause). MATLAB partial port regressed nominal -> A/B before trusting.
        self._CH_CLEAN = os.environ.get("PLASMC_CH_CLEAN", "0") == "1"

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
        self._K_R = np.diag([float(os.environ.get("PLASMC_KR_ROLL",  "1.5")),
                             float(os.environ.get("PLASMC_KR_PITCH", "1.5")),
                             float(os.environ.get("PLASMC_KR_YAW",   "0.5"))])   # 2/2/2->1.5/1.5/0.5 (2026-06-12 Combo bake): MATLAB SO(3) parity

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
        self._h_d_noS = []             # h_d minus the s_dot term (transport+descent) -> dh_d drops s_ddot
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
        self._marker_extent = []   # MARKER_EXTENT_PX per step (proximity / terminal-hold trigger)
        # FoV-cone diagnostics
        self._rho_fov_log = []
        self._d_min_fov_log = []
        self._theta_cone_log = []
        self._theta_current_log = []
        self._cbf_state = {}       # persistent cbf2 state (former _lw_*); see cbf_visibility.cbf2_filter
        self._theta_safe = None    # cbf2 Phase-1 safe lean vector (Fix B: direct->rd3)
        self._w_u = []
        self._B_T = []
        self._u = []

        # Time
        self._t = []
        self._dt = []
        self._t0 = self._time.perf_counter()

    def run(self):
        while self._img_node.is_alive() and self._STAY_OPEN:
            if self._img_node.FEATURE_IS_VISIBLE:

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
            self._dt.append(self._t[-1] - self._t[-2])

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
        self._s_e.append(self._s[-1] - self._s_d)

        # Normalized pixel error (sensor-half normalization)
        s_e_n = self._s_e[-1][:2] / self._p_10
        self._s_e_n.append(s_e_n)

        if self._combined_barrier:
            # === COMBINED-BARRIER: position barrier zeta_r enters the sliding surface (no back-map) ===
            # measured centroid rate s_dot (= d s_e[:2]/dt, s_d const) — the h_d feedforward AND
            # the r_bar_e rate (rel deg 2); kappa absorbs the 1/z-inflated s_ddot (dropped from dh_d).
            if len(self._s_e) > 1 and len(self._dt) > 0 and self._dt[-1] > 1e-6:
                self._s_dot_deque.append((self._s_e[-1][:2] - self._s_e[-2][:2]) / self._dt[-1])
            else:
                self._s_dot_deque.append(np.zeros(2))
            self._s_dot_deque.popleft()
            s_dot_meas = smooth4(self._s_dot_deque)
            self._s_dot_meas.append(s_dot_meas)
            # position barrier on r_bar_e = s_e_n (= s_e/p_10, FoV-normalized) with funnel p_r
            r_bar_e  = self._s_e_n[-1]
            dr_bar_e = s_dot_meas / self._p_10                       # measured rate (rel deg 2)
            S_r = np.zeros(2); zeta_r = np.zeros(2); dzeta_r = np.zeros(2)
            for _i in range(2):
                S_r[_i]     = float(np.clip(r_bar_e[_i] / self._p_r[-1][_i], -1.0 + S_MARGIN, 1.0 - S_MARGIN))
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
            # Trapezoidal integration of normalized error + anti-windup
            if len(self._is_e_n) == 0:
                self._is_e_n.append(np.zeros(2))
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
        self._h.append(h)

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
            self._h_d.append(np.concatenate([self._s_dot_meas[-1], [0.0]]) + h_d_noS)
        elif self._CH_CLEAN:
            # Consistent c_h kinematics correction (manuscript §II, 2026-06-11; feedback_ch_kinematics_correction).
            # Desired flow DROPS the w×s rotational feedforward — the corrected c-term carries the rotation
            # via -ψ̇_b·(ê3×h) instead (see PLASMC). Removing w×s here is the OTHER half of the consistent
            # change (MATLAB C_SIMPLE only changed c -> convention-mixed regression; this does both). Directly
            # de-dominates h_d (which the a_u-outward diagnosis showed is ~all cross(w_i,s) in the overshoot).
            self._h_d.append(self._ds_d[-1] + h_ref_eff * self._s[-1][:3])
        else:
            self._h_d.append(
                self._ds_d[-1]
                + cross_ws
                + (h_ref_eff - np.dot(cross_ws, e3)) * self._s[-1][:3]
            )
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
                ratio = np.abs(float(self._S[-1][idx, idx])) * np.sign(ratio)  # hold last-good magnitude, trust current sign
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
        _hd_src = self._h_d_noS if self._combined_barrier else self._h_d
        if len(_hd_src) > 1:
            self._dh_d_deque.append((_hd_src[-1] - _hd_src[-2]) / self._dt[-1])
            self._dh_d_deque.popleft()
        self._dh_d.append(np.clip(smooth4(self._dh_d_deque), -DH_D_MAX, DH_D_MAX))

    def PLASMC(self):
        """Middle-loop adaptive SMC -> body-frame acceleration command a_u."""
        t = self._t[-1] - self._t0

        # Integral of zeta (trapezoidal) with anti-windup
        if len(self._izeta) == 0:
            self._izeta.append(np.zeros(N_DIM))
        else:
            new_int = (self._izeta[-1]
                       + self._dt[-1] * 0.5 * (self._zeta[-1] + self._zeta[-2]))
            # Anti-windup: per-COMPONENT clamp (matches MATLAB visualControl_IBVS
            # _adaptive.m:393-394). Was norm-clamp, which saturated ~30% earlier
            # on 3-vectors at the limit (norm=√3·5 vs per-axis 5 each).
            new_int = np.clip(new_int, -self._izeta_clamp, self._izeta_clamp)
            self._izeta.append(new_int)

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
        self._dw.append(smooth4(self._dw_deque))

        # Sliding surface. Back-mapped (default): sigma = zeta_h + Omega*int(zeta_h).
        # Combined-barrier: lateral sigma_k = zeta_h_k + chi_r*zeta_r_k (PD), descent
        # sigma_3 = zeta_h_3 + chi_z*int(zeta_h_3) (PI, chi_z = Omega_z, unchanged). chi_zeta_aug =
        # chi*zeta_dot_aug is the measured drift folded into u_eq/Theta (= Omega*zeta_h back-mapped;
        # = [chi_r*dzeta_r; Omega_z*zeta_h_3] combined). Relative degree 1 preserved.
        if self._combined_barrier:
            _zr = self._zeta_r[-1]
            _sig = self._zeta[-1].copy()
            _sig[0] += self._chi_r[0] * _zr[0]
            _sig[1] += self._chi_r[1] * _zr[1]
            _sig[2] += self._Omega[2, 2] * self._izeta[-1][2]
            self._sigma.append(_sig)
            chi_zeta_aug = np.array([self._chi_r[0] * self._dzeta_r[-1][0],
                                     self._chi_r[1] * self._dzeta_r[-1][1],
                                     self._Omega[2, 2] * self._zeta[-1][2]])
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
            psi_dot_b = (float(self._w[-1][1]) * np.sin(_phi) + _r * np.cos(_phi)) / max(abs(np.cos(_th)), 0.2)
            c = (- psi_dot_b * np.cross(e3, self._h[-1])
                 - np.dot(self._h[-1], e3) * self._h[-1]
                 - self._dh_d[-1])
        else:
            c = (np.cross(self._dw[-1], self._s[-1][:3])
                 + np.cross(w, np.cross(w, self._s[-1][:3]))
                 + 2 * np.cross(w, self._h[-1])
                 - (np.dot(self._h[-1] + np.cross(w, self._s[-1][:3]), e3)) * self._h[-1]
                 - self._dh_d[-1])

        # Theta matrix and its Frobenius norm
        # Theta = [-c + S*dp - G\(chi*zeta_dot_aug), eye(3)]  (chi_zeta_aug set at the surface above)
        vector = (- c
                  + self._S[-1] @ self._dp[-1]
                  - np.linalg.solve(self._G[-1], chi_zeta_aug))
        Theta = np.hstack([vector.reshape(-1, 1), np.eye(N_DIM)])
        self._theta.append(np.linalg.norm(Theta, ord='fro'))

        # Adaptive-gain (translational) update via RK5
        # MATLAB: dkappa/dt = Theta_norm * N * G * |sigma| - N * P * kappa
        # Freeze κ on axes where Singhal outlier containment fired (funnel-breach glitch — the clipped
        # G/σ would drive κ up via the barrier singularity, G→∞ near the funnel edge).
        # NOTE (2026-06-10): a 2nd "sustained-high-θ" κ-freeze trigger was prototyped + DROPPED — it is
        # mis-targeted. At E_Z=0.5 κ_z ratchets up during the MODERATE-θ stretches (median ~3-7, with
        # the freeze OFF) via large G·|σ| at close range — NOT the brief high-θ spikes the trigger
        # chased (θ>50 on only ~19% of frames). No θ threshold (50 or 200) catches it. The lever for
        # the E_Z=0.5 κ-bound is P_z (κ_eq ∝ 1/P leakage). See memory feedback_theta_norm_klt_drift.
        if len(self._dt) > 0:
            new_kappa = RK5(self._kappaSolver, t, self._kappa[-1],
                            [self._sigma[-1], self._theta[-1]], self._dt[-1])
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
               - self._theta[-1] * np.diag(sat_sigma) @ self._G[-1] @ self._kappa[-1]
               + self._G[-1] @ (- c + self._S[-1] @ self._dp[-1])
               - chi_zeta_aug)
        self._a_v.append(a_v)

        a_u = - np.linalg.solve(self._G[-1], a_v)
        # NOTE: the legacy |a_u|>100 abort was removed. PX4 saturates attitude-
        # rate setpoints internally to physical limits (~±220 deg/s); an
        # over-large a_u from a noisy startup PID firing just produces a
        # brief wobble, not a crash. Keeping the controller alive lets the
        # SMC adapt and recover instead of killing the run permanently.
        # _warmup_remaining is retained as a state variable for future use
        # but no longer gates anything here.
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
        e_a_raw = self._s[-1][3] - self._s_d[3]
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
        _w_max = float(os.environ.get("PLASMC_W_U_MAX", "1.0"))
        _psid_rate = float(os.environ.get("PLASMC_YAW_PSID_RATE", "1.0")) * _w_max   # 0.7->1.0: un-throttle psi_d to full W_U_MAX (converge large initial yaw, 2026-06-08)
        _ua_psid = float(np.clip(u_a, -_psid_rate, _psid_rate))
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

    def _kappaSolver(self, _, kappa, X):
        sigma = X[0]
        theta_norm = X[1]
        return theta_norm * self._N @ self._G[-1] @ np.abs(sigma) - self._N @ self._P @ kappa

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
        # NOTE: a_u lives in the V frame; MATLAB uses I_R_V = rotz(yaw) here,
        # Python uses the full body DCM R — open parity item D1 in
        # docs/CONTROLLER_PARITY.md (zero difference when level; grows with tilt).
        I_a_raw = R @ self._a_u[-1] - np.array([0.0, 0.0, g])
        self._I_a_raw.append(I_a_raw.copy())

        # ---- Full MATLAB FoV-margin cone (visualControl_IBVS_adaptive.m:443-469) ----
        I_a = I_a_raw.copy()

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
        d_min_fov = 0.0
        try:
            fp_list = self._img_node._feature_pts
            if len(fp_list) > 0:
                raw_corners = np.asarray(fp_list[-1][1])   # (4, 2) — (u, v) top-left
                cx, cy = self._img_node.center
                u_centered = raw_corners[:, 0] - cx
                v_centered = raw_corners[:, 1] - cy
                d_corner_x = rho_fov_curr[0] - np.abs(u_centered)   # (4,)
                d_corner_y = rho_fov_curr[1] - np.abs(v_centered)   # (4,)
                d_min_fov = max(float(np.min(np.concatenate([d_corner_x, d_corner_y]))), 0.0)
        except (IndexError, AttributeError, ValueError, TypeError):
            d_min_fov = 0.0   # fall back to "no extra tilt allowed"

        # 4) Cone angle = current tilt + tilt-headroom-before-the-marker-exits, capped.
        focal_px = float(self._img_node.focal[0])
        foc = np.asarray(self._img_node.focal, float)            # [fx, fy]
        funnel_mode = self._funnel_mode
        if funnel_mode in ("cone0", "cbf1"):
            # === L_omega camera-plane CBF (docs/CBF_visibility.pdf), per-cycle steps ===
            # The tilt->feature coupling is the rotational interaction matrix L_omega at
            # the MEASURED camera image point (tangent units) — exact, depth-free, no
            # virtual frame. The headroom is the binding-axis margin / ||L_omega row||.
            tau = float(os.environ.get("CBF_TAU", "0.3")) if funnel_mode == "cbf1" else 0.0
            theta_cone = float(min(theta_current + np.arctan(d_min_fov / focal_px),
                                   self._theta_cap))   # d_min fallback; overwritten below if cr available
            try:
                rc = np.asarray(self._img_node._feature_pts[-1][1], float)   # (N,2) u-v
                ct = (rc - np.asarray(self._img_node.center, float)) / foc    # corners, tangent
                cr = ct.mean(0)                                              # (step 2) ^C r_hat centroid
                delta = 0.5 * (ct.max(0) - ct.min(0))                        # visible-set half-extent
                x, y = float(cr[0]), float(cr[1])
                L_w = np.array([[x * y, -(1.0 + x * x)], [1.0 + y * y, -x * y]])   # (step 3) L_omega
                d = np.zeros(2); ddelta = 0.0
                if funnel_mode == "cbf1" and len(self._dt) > 0 and self._dt[-1] > 1e-6 \
                        and getattr(self, "_lw_cr_prev", None) is not None:
                    # (step 3) exogenous drift d = cr_dot_obs - L_omega @ omega_rp (strip our tilt)
                    w_rp = np.asarray(self._w[-1][:2], float) if len(self._w) > 0 else np.zeros(2)
                    d_raw = (cr - self._lw_cr_prev) / self._dt[-1] - L_w @ w_rp
                    ema = float(os.environ.get("CBF_DMIN_EMA", "0.3"))
                    self._lw_d = (1 - ema) * getattr(self, "_lw_d", np.zeros(2)) + ema * d_raw
                    d = self._lw_d
                    ddelta = max((float(np.linalg.norm(delta)) - getattr(self, "_lw_dprev",
                                  float(np.linalg.norm(delta)))) / self._dt[-1], 0.0)
                self._lw_cr_prev = cr.copy(); self._lw_dprev = float(np.linalg.norm(delta))
                # (step 4) predicted margin m = phi_max - |cr + tau*d| - delta - tau*ddelta
                m = np.maximum(np.asarray(self._p_10, float) - np.abs(cr + tau * d) - delta - tau * ddelta, 0.0)
                headroom = float(np.min(m / (np.linalg.norm(L_w, axis=1) + 1e-9)))   # rad
                theta_cone = float(min(theta_current + headroom, self._theta_cap))
            except (IndexError, AttributeError, ValueError, TypeError):
                pass   # keep the d_min fallback above
        else:
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
        # FUNNEL_MODE selects how the lateral accel is constrained for visibility.
        #   "cone" (default): MATLAB magnitude clamp (two-sided ball on ‖a_xy‖; strangles recovery).
        #   "cone0"/"cbf1": directional clamp w/ the L_omega headroom above (lean-magnitude form).
        #   "cbf2": exact camera-frame theta-QP (image-axis tilt; retires the lean approximation).
        self._theta_safe = None        # Fix B: cbf2 Phase-1 safe lean vector for direct->rd3; None => accel path
        if funnel_mode == "cbf2":
            # === Exact camera-frame theta-QP (docs/CBF_visibility.pdf — the literal QP) ===
            # Extracted verbatim into cbf_visibility.cbf2_filter so the offline
            # validators (tools/validate_cbf.py, tools/replay_cbf.py) run the EXACT
            # live code path. The barrier, two-phase δ, and Phase-2 fallback all live
            # there; this site only marshals the controller state into pure args.
            try:
                corners = self._img_node._feature_pts[-1][1]
            except (IndexError, AttributeError, TypeError):
                corners = None
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
        else:
            a_xy_lim = abs(I_a[2]) * np.tan(theta_cone)
            t_hat = None
            if funnel_mode in ("cone0", "cbf1") and len(self._s) > 0:
                s_xy = np.asarray(self._s[-1][:2], float)
                s_n = float(np.linalg.norm(s_xy))
                if s_n > 1e-6:
                    v = s_xy / s_n
                    # image->inertial toward-target map (SIGN-CAL gate; default validated cos.I_a=0.84)
                    if os.environ.get("CONE0_SWAP", "0") == "1":
                        v = v[::-1]
                    v = v * np.array([float(os.environ.get("CONE0_SIGN_X", "1.0")),
                                      float(os.environ.get("CONE0_SIGN_Y", "1.0"))])
                    cz, sz = np.cos(yaw_c), np.sin(yaw_c)
                    t_hat = np.array([cz * v[0] - sz * v[1], sz * v[0] + cz * v[1]])  # Rz(yaw)·v
            if t_hat is not None:
                a_par = float(I_a[:2] @ t_hat)        # >0 = toward marker (free)
                if a_par < -a_xy_lim:                 # outward faster than allowed
                    I_a[:2] = I_a[:2] + (-a_xy_lim - a_par) * t_hat
            else:
                a_xy_n = np.linalg.norm(I_a[:2])      # magnitude clamp (cone / fallback)
                if a_xy_n > a_xy_lim and a_xy_n > 1e-9:
                    I_a[:2] = a_xy_lim * I_a[:2] / a_xy_n
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

        # Hard clamp on body-rate command magnitude. LK optical flow has a
        # ~15 px tracking window; at our 540 px focal length and 60 Hz
        # image rate, a body rate of ~1.7 rad/s = 100°/s already pushes
        # corner motion to the LK limit. Clamping at 1.0 rad/s (= 57°/s)
        # gives a safety margin and prevents transient attitude-error
        # spikes from breaking optical-flow tracking (2026-05-21 finding).
        # Env-overridable; default 1.0 rad/s.
        w_max = float(os.environ.get("PLASMC_W_U_MAX", "1.0"))
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
            "funnel_mode": self._funnel_mode,
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
