"""
Generate publication-quality VDF-ASMC internals plots from Multi_init_cond datasets.

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
# Figure 1: combined funnels (1x4) -- image-feature funnel + optic-flow funnel
# ======================================================================
fig, axes = plt.subplots(1, 4, figsize=(18.0, 4.6))

# col 1: image-feature funnel -- r_bar_e (2 lateral axes) inside +/-p_r(t)
ax = axes[0]
ax.fill_between(t, p_r[0], -p_r[0], color="orange", alpha=0.16,
                label=r"$\pm\boldsymbol{p}_r(t)$")
ax.plot(t,  p_r[0], color="orange", lw=1.0)
ax.plot(t, -p_r[0], color="orange", lw=1.0)
for k, c in zip(range(2), ["C0", "C2"]):
    ax.plot(t, r_bar_e[k], color=c, lw=1.4,
            label=fr"$\bar r_{{\mathrm{{e}},{axis_lbl[k][1]}}}(t)$")
ax.set_xlabel(r"$t$ [s]", fontsize=20, labelpad=4)
ax.set_ylabel(r"$\bar{\boldsymbol{r}}_{\mathrm{e}}(t)$  [FoV units]", fontsize=20, labelpad=4)
ax.set_title("Image-Feature Funnel", fontsize=20)
ax.tick_params(labelsize=16)
ax.locator_params(axis="x", nbins=4)
ax.legend(loc="upper right", fontsize=13)

# cols 2-4: optic-flow funnel -- h_e_k inside +/-p_h_k(t)
for k in range(3):
    ax = axes[k + 1]
    ax.fill_between(t, p_h[k], -p_h[k], color="orange", alpha=0.16,
                    label=r"$\pm\boldsymbol{p}_h(t)$" if k == 0 else None)
    ax.plot(t,  p_h[k], color="orange", lw=1.0)
    ax.plot(t, -p_h[k], color="orange", lw=1.0)
    ax.plot(t, h_e[k], color="C0", lw=1.4,
            label=r"$h_{\mathrm{e},k}(t)$" if k == 0 else None)
    ax.set_xlabel(r"$t$ [s]", fontsize=20, labelpad=4)
    ax.set_ylabel(fr"$h_{{\mathrm{{e}},{axis_lbl[k][1]}}}$ [1/s]", fontsize=20, labelpad=4)
    ax.set_title(fr"Optic-Flow Funnel ({axis_lbl[k]})", fontsize=20)
    ax.tick_params(labelsize=16)
    ax.locator_params(axis="x", nbins=4)
axes[1].legend(loc="upper right", fontsize=13)

fig.suptitle("Dual-Funnel Invariance under VDF-ASMC", fontsize=24, y=1.0)
fig.tight_layout(pad=0.5)
fig.savefig(f"{OUT}/plasmc_funnel_combined.pdf", bbox_inches="tight", pad_inches=0.03)
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
fig.savefig(f"{OUT}/plasmc_sliding.pdf", bbox_inches="tight", pad_inches=0.02)
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
fig.savefig(f"{OUT}/plasmc_adaptive_gain.pdf", bbox_inches="tight", pad_inches=0.03)
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
fig.savefig(f"{OUT}/plasmc_thrust_accel.pdf", bbox_inches="tight", pad_inches=0.02)
plt.close(fig)

print("Wrote 4 VDF-ASMC internals plots (CBF-architecture rebuild) to:", OUT)
