"""
Generate publication-quality VISTA internals plots from Multi_init_cond datasets.

Rebuilt 2026-06-25 for the CURRENT visibility-CBF + combined-barrier architecture
(the previous version targeted the superseded "Approach 2" funnel-margin / cone-clamp
design and read fields run_simulation no longer logs: rho_fov_log, p_2, a time-series
kappa). Now driven by the internals logs added to run_simulation.m:
  kappa_log, kappa_a_log, theta_cone_log, s_e_log, p_r_log, p_h_log, dist_log.

Outputs PDFs into Figures/generated/:
  plasmc_funnel_combined.pdf — image-feature funnel (r_bar_e vs +/-p_r) + optic-flow
                               funnel (h_e vs +/-p_h) per axis (1x4).
  plasmc_sliding.pdf         — sliding surface sigma vs boundary layer +/-E.
  plasmc_adaptive_gain.pdf   — kappa(t) overlaid with the injected disturbance it adapts
                               against (twin axis) + yaw gain kappa_a(t).
  plasmc_thrust_accel.pdf    — total thrust + lateral acceleration vs CBF tilt-cone bound.

Representative case: IC2 = [2,2,-5] on Sinusoidal (Case 3) trajectory (index 1, 0-based).
NOTE for the manuscript: the funnel figure now shows the centroid error r_bar_e in the
p_r funnel (current theory, manuscript eq. position barrier), NOT the 4 feature points in
an rho_fov box (old Approach 2). The caption (which still says "four feature points / p_1")
should be reconciled to "centroid r_bar_e / p_r".
"""
import os
import numpy as np
import scipy.io as sio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["cmr10", "Computer Modern Roman", "DejaVu Serif"],
    "mathtext.fontset": "cm",
    "axes.formatter.use_mathtext": True,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "lines.linewidth": 1.4,
})

from pathlib import Path
ROOT = str(Path(__file__).resolve().parent.parent)
DATA = f"{ROOT}/MATLAB/Datasets/MultiInit/Sinusoidal_multi_init.mat"
OUT  = f"{ROOT}/Soft_Precise_Landing/Figures/generated"
os.makedirs(OUT, exist_ok=True)


def safe_savefig(fig, target_path, **kwargs):
    """Save to a unique temp file, then replace the target -- a target open
    in a PDF viewer (Acrobat, etc.) locks the path against a direct write."""
    tmp_path = f"{os.path.dirname(target_path)}/.tmp_{os.getpid()}_{os.path.basename(target_path)}"
    fig.savefig(tmp_path, **kwargs)
    try:
        os.replace(tmp_path, target_path)
        print(f"Wrote {target_path}")
    except PermissionError:
        print(f"WARNING: {target_path} is locked (likely open in a viewer) -- "
              f"wrote {tmp_path} instead. Close it and re-run.")


m = sio.loadmat(DATA, squeeze_me=True, struct_as_record=False)
run = m["results"][1]   # IC2 = [2,2,-5]
d = run.data
N = int(d.idx) if int(d.idx) > 0 else d.tRange.shape[0]
t = d.tRange[:N]


# --- current internals logs ---
sigma   = d.sigma[:, :N]          # 3 x N  sliding surface
kappa   = d.kappa_log[:, :N]      # 3 x N  translational adaptive gain
kappa_a = d.kappa_a_log[:N]       # N      yaw adaptive gain
h_e     = d.V_h_e[:, :N]          # 3 x N  optic-flow error
p_h     = d.p_h_log[:, :N]        # 3 x N  optic-flow funnel envelope
s_e     = d.s_e_log[:, :N]        # 2 x N  lateral centroid feature error
p_r     = d.p_r_log[:, :N]        # 2 x N  image-feature funnel envelope
# per-axis lumped disturbance the adaptive gain rejects:
#   d_bar = beta_min^-1 [beta-1; d_h];  Y=[v|I3];  (Y d_bar)_k = (v_k(beta-1)+d_h,k)/beta_min
d_h     = d.d_h_log[:, :N]        # 3 x N  reconstructed optic-flow disturbance
beta    = d.beta_log[:N]          # N      depth scale 1/z
v_reg   = d.v_log[:, :N]          # 3 x N  regressor first column
beta_min = float(beta.min())
Yd_axis = (v_reg * (beta - 1.0) + d_h) / beta_min     # 3 x N  per-axis (Y d_bar)_k
from scipy.signal import savgol_filter
Yd_mag = savgol_filter(np.abs(Yd_axis), 151, 1, axis=1)   # smooth the h_e finite-diff spikes
B_T_cd  = d.B_T_cd[:N]            # N      total thrust command [N]
I_a_cd  = d.I_a_cd[:, :N]         # 3 x N  desired inertial acceleration
theta_cone = d.theta_cone_log[:N] # N      CBF tilt-cone half-angle [rad]

phi_max = np.ravel(d.P.phi_max)[:2]            # [2] half-FoV tangent units (s-axis order)
r_bar_e = s_e / phi_max[:, None]               # normalized lateral position error
E_diag  = np.ravel(d.P.E) if hasattr(d.P, "E") else np.array([1.0, 1.0, 0.5])
T_max   = float(d.T_max)
T_min   = float(d.T_min)

axis_lbl = [r"$x$", r"$y$", r"$z$"]

# ======================================================================
# Figure 1: normalized prescribed-performance envelopes (1x2)
#   Rebuilt 2026-09-11 at user request: the raw-unit 1x4 (r_bar_e vs +/-p_r,
#   h_e vs +/-p_h per axis) let the envelope's own shrinking dominate the
#   vertical scale, burying the actual error near zero -- weak evidence for
#   a strong result. Dividing error by envelope collapses both funnels to a
#   fixed +/-1 band: the reviewer sees the prescribed-performance envelope
#   held directly, with no per-panel unit or scale to reconcile. Paper frames
#   this as prescribed-performance envelope preservation (Theorem thm:precision),
#   not a separate "funnel invariance" result -- title updated to match.
# ======================================================================
pos_ratio = r_bar_e / p_r     # 2 x N, in [-1, 1]: image-position error / its envelope
vel_ratio = h_e / p_h         # 3 x N, in [-1, 1]: optic-flow error / its envelope

# Panel (c): funnel-compatibility ratio C_k = rho_nu,k / (rho_nu,z * |s_k|), k in {x,y}.
# s_k is the RAW (not error) normalized lateral image position -- V_X_DS rows 1:2 are
# cs.V_s_i(1:2), the controller-recovered s_x, s_y (matches s_e_log's source, so it's
# the same internal signal the rest of this figure already uses).
s_xy   = d.V_X_DS[0:2, :N]                    # 2 x N
compat = p_h[0:2] / (p_h[2:3] * (np.abs(s_xy) + 1e-9)) # 2 x N: C_x, C_y (eps guards s_k==0)

print("Panel 1 (position) peak |ratio| per axis:", np.max(np.abs(pos_ratio), axis=1))
print("Panel 2 (velocity) peak |ratio| per axis:", np.max(np.abs(vel_ratio), axis=1))
print("Panel 2 (velocity) terminal ratio per axis:", vel_ratio[:, -1])
print("Panel 3 (compatibility) min ratio per axis:", np.nanmin(compat, axis=1))

fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.6))

ax = axes[0]
for k, c in zip(range(2), ["C0", "C2"]):
    ax.plot(t, pos_ratio[k], color=c, lw=1.4,
            label=fr"$e_{{p,{axis_lbl[k][1]}}}/(\varphi_{{{axis_lbl[k][1]},\max}}\rho_{{p,{axis_lbl[k][1]}}})$")
# Zoomed to the trajectories themselves (peak |ratio| ~ 0.045, printed above)
# so the near-origin evolution is visible -- the +/-1 envelope bound and its
# shading are dropped here since they fall far outside this view and would
# render as a flat, uninformative background tint; the margin below the
# envelope is instead reported in the caption/printed diagnostics.
_pmax = float(np.max(np.abs(pos_ratio)))
_pmax = _pmax if _pmax > 0 else 1.0
ax.set_ylim(-1.2 * _pmax, 1.2 * _pmax)
ax.set_xlabel(r"$t$ [s]", fontsize=20, labelpad=4)
ax.set_ylabel("normalized image-position error", fontsize=17, labelpad=4)
ax.set_title("(a) Position Error", fontsize=20)
ax.tick_params(labelsize=16)
ax.locator_params(axis="x", nbins=4)
ax.legend(loc="upper right", fontsize=13)

ax = axes[1]
for k, c in zip(range(3), ["C0", "C2", "C3"]):
    ax.plot(t, vel_ratio[k], color=c, lw=1.4,
            label=fr"$e_{{\nu,{axis_lbl[k][1]}}}/\rho_{{\nu,{axis_lbl[k][1]}}}$")
_vmax = float(np.max(np.abs(vel_ratio)))
_vmax = _vmax if _vmax > 0 else 1.0
ax.set_ylim(-1.2 * _vmax, 1.2 * _vmax)
ax.set_xlabel(r"$t$ [s]", fontsize=20, labelpad=4)
ax.set_ylabel("normalized optic-flow error", fontsize=17, labelpad=4)
ax.set_title("(b) Velocity Error", fontsize=20)
ax.tick_params(labelsize=16)
ax.locator_params(axis="x", nbins=4)
ax.legend(loc="upper right", fontsize=13)

# --- Panel (c): funnel-compatibility ratio C_k(t), sufficient condition C_k > 1 ---
# Log y-axis: C_k = rho_nu,k/(rho_nu,z*|s_k|) is singular whenever the raw lateral
# position s_k crosses zero (happens naturally on an oscillatory target -- s_k is a
# signed position, not an error, so it passes through 0 every half-cycle). Those
# crossings are not a real margin signal, just 1/|s_k| blowing up; log-scale
# compresses them to readable peaks while keeping the dimensionless ratio and the
# C_k>1 threshold (now log C_k>0) intact.
ax = axes[2]
ax.axhline(1.0, color="0.35", lw=1.0, ls="--")
for k, c in zip(range(2), ["C0", "C2"]):
    ax.plot(t, compat[k], color=c, lw=1.4,
            label=fr"$\mathcal{{C}}_{axis_lbl[k][1]}(t)$")
ax.set_yscale("log")
ax.set_xlabel(r"$t$ [s]", fontsize=20, labelpad=4)
ax.set_ylabel(r"$\mathcal{C}_k(t)=\rho_{\nu,k}/(\rho_{\nu,z}\bar s_k)$", fontsize=15, labelpad=4)
ax.set_title("(c) Funnel Compatibility", fontsize=20)
ax.tick_params(labelsize=16)
ax.locator_params(axis="x", nbins=4)
ax.legend(loc="upper right", fontsize=13)

fig.suptitle("Prescribed-Performance Preservation and Funnel Compatibility", fontsize=20, y=1.0)
fig.tight_layout(pad=0.5)
safe_savefig(fig, f"{OUT}/plasmc_funnel_combined.pdf", bbox_inches="tight", pad_inches=0.03)
plt.close(fig)

# ======================================================================
# Figure 2: sliding surface sigma vs boundary layer +/-E
# ======================================================================
fig, ax = plt.subplots(figsize=(10.5, 4.0))
for k, c in zip(range(3), ["C0", "C2", "C3"]):
    ax.plot(t, sigma[k], color=c, label=fr"$\sigma_{axis_lbl[k][1]}$")
    ax.axhline(+E_diag[k], color=c, lw=0.7, ls=":", alpha=0.6)
    ax.axhline(-E_diag[k], color=c, lw=0.7, ls=":", alpha=0.6)
ax.axhline(0.0, color="k", lw=0.6, ls=":")
ax.set_xlabel(r"$t$ [s]", labelpad=4, fontsize=20)
ax.set_ylabel(r"$\boldsymbol{\sigma}(t)$", labelpad=4, fontsize=20)
ax.locator_params(axis="x", nbins=4)
ax.tick_params(pad=1, labelsize=16)
ax.legend(loc="upper right", ncol=3, fontsize=14,
          title=r"dotted: boundary layer $\pm\mathcal{E}$", title_fontsize=11)
fig.suptitle("Sliding-Surface Evolution", fontsize=24, y=0.99)
fig.tight_layout(pad=0.3)
safe_savefig(fig, f"{OUT}/plasmc_sliding.pdf", bbox_inches="tight", pad_inches=0.02)
plt.close(fig)

# ======================================================================
# Figure 3: per-axis adaptive gain kappa_k vs the per-axis disturbance it rejects
#   kappa is a 3-vector (per-axis regressor norm); each kappa_k bounds its OWN
#   (Y d_bar)_k = (v_k(beta-1)+d_h,k)/beta_min, not a shared scalar. The
#   disturbance grows ~1/z near the deck; the funnel/barrier absorbs that growth
#   so kappa stays bounded. kappa_alpha (yaw): Case 3 (Sinusoidal) has no target
#   yaw, so kappa_alpha decays under leakage -- no spurious adaptation.
# ======================================================================
fig, axes = plt.subplots(2, 2, figsize=(11.5, 7.6))
panels = [(axes[0, 0], 0, "x", "C0"),
          (axes[0, 1], 1, "y", "C2"),
          (axes[1, 0], 2, "z", "C3")]
for ax, k, lab, col in panels:
    # kappa_k and its per-axis disturbance on the SAME (log) axis -- true values,
    # both visible despite kappa ~0.1 << |(Y d_bar)_k| ~ O(1-30) (the funnel/barrier
    # G_h supplies the rest of the rejection, so kappa stays small).
    ax.semilogy(t, kappa[k], color=col, lw=1.8, label=fr"$\kappa_{{{lab}}}$")
    ax.semilogy(t, np.clip(Yd_mag[k], 1e-3, None), color="0.35", lw=1.2, ls="--",
                label=fr"$|(Y\bar{{\boldsymbol{{d}}}})_{{{lab}}}|$")
    ax.set_xlabel(r"$t$ [s]", fontsize=17, labelpad=3)
    ax.set_ylabel(fr"$\kappa_{{{lab}}},\ |(Y\bar{{d}})_{{{lab}}}|$", fontsize=17, labelpad=3)
    ax.tick_params(axis="both", labelsize=13)
    ax.locator_params(axis="x", nbins=4)
    ax.set_title(fr"${lab}$-axis", fontsize=18)
    ax.legend(loc="upper right", fontsize=12, framealpha=0.85)

# yaw panel
axes[1, 1].plot(t, kappa_a, color="C4", lw=1.7, label=r"$\kappa_\alpha$")
axes[1, 1].set_xlabel(r"$t$ [s]", fontsize=17, labelpad=3)
axes[1, 1].set_ylabel(r"$\kappa_\alpha(t)$", fontsize=17, labelpad=3)
axes[1, 1].set_title(r"yaw (no target yaw on Case 3)", fontsize=18)
axes[1, 1].tick_params(labelsize=13)
axes[1, 1].locator_params(axis="x", nbins=4)
axes[1, 1].legend(loc="upper right", fontsize=11)

fig.suptitle("Adaptive Gain Rejection of the Per-Axis Disturbance", fontsize=22, y=1.0)
fig.tight_layout(pad=0.5)
fig.subplots_adjust(wspace=0.42, hspace=0.42)
safe_savefig(fig, f"{OUT}/plasmc_adaptive_gain.pdf", bbox_inches="tight", pad_inches=0.03)
plt.close(fig)

# ======================================================================
# Figure 4: thrust + lateral acceleration vs CBF tilt-cone bound
# ======================================================================
a_xy_norm = np.linalg.norm(I_a_cd[:2], axis=0)
cone_limit = np.abs(I_a_cd[2]) * np.tan(theta_cone)
fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.3))
axes[0].plot(t, B_T_cd, color="C0", label=r"$\,^\mathcal{B}T_u(t)$")
axes[0].axhline(T_max, color="r", lw=0.8, ls="--", label=r"$T_{\max}$")
axes[0].axhline(T_min, color="b", lw=0.8, ls="--", label=r"$T_{\min}$")
axes[0].set_xlabel(r"$t$ [s]", labelpad=4, fontsize=20)
axes[0].set_ylabel("total thrust [N]", labelpad=4, fontsize=20)
axes[0].set_title("Thrust Command", fontsize=20)
axes[0].locator_params(axis="x", nbins=4)
axes[0].tick_params(pad=1, labelsize=16)
axes[0].legend(loc="upper right", fontsize=14)

axes[1].plot(t, a_xy_norm, color="C0", label=r"$\|\,^\mathcal{I}\boldsymbol{a}_{\mathrm{d},xy}\|$")
axes[1].plot(t, cone_limit, color="r", lw=0.9, ls="--",
             label=r"$|\,^\mathcal{I}a_{\mathrm{d},z}|\tan\theta_\mathrm{cone}(t)$")
axes[1].set_xlabel(r"$t$ [s]", labelpad=4, fontsize=20)
axes[1].set_ylabel(r"lateral accel [m/s$^2$]", labelpad=4, fontsize=20)
axes[1].set_title("Cone-Constrained Lateral Acceleration", fontsize=20)
axes[1].locator_params(axis="x", nbins=4)
axes[1].tick_params(pad=1, labelsize=16)
axes[1].legend(loc="upper right", fontsize=14)

fig.suptitle("Thrust and Lateral Acceleration", fontsize=24, y=0.99)
fig.tight_layout(pad=0.3)
fig.subplots_adjust(wspace=0.3)
safe_savefig(fig, f"{OUT}/plasmc_thrust_accel.pdf", bbox_inches="tight", pad_inches=0.02)
plt.close(fig)

print("Wrote 4 VISTA internals plots (CBF-architecture rebuild) to:", OUT)
