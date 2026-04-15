"""
Generate publication-quality PLASMC internals plots from Multi_init_cond datasets.

Outputs PDFs into Figures/generated/:
  plasmc_outer_funnel.pdf   — image error vs outer visibility funnel p_1
  plasmc_inner_funnel.pdf   — optic-flow error vs inner funnel p_2
  plasmc_sliding.pdf        — sliding surface sigma per axis
  plasmc_adaptive_gain.pdf  — kappa(t) and kappa_a(t)
  plasmc_thrust_accel.pdf   — total thrust + lateral acceleration vs cone bound

Representative case: IC4 = [2,2,-5] on Sinusoidal trajectory (index 3, 0-based).
"""
import os
import numpy as np
import scipy.io as sio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 9,
    "axes.labelsize": 9,
    "axes.titlesize": 10,
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "lines.linewidth": 1.2,
})

from pathlib import Path
ROOT = str(Path(__file__).resolve().parent.parent)
DATA = f"{ROOT}/MATLAB/Multi_init_cond/Datasets/Sinusoidal_multi_init.mat"
OUT  = f"{ROOT}/Soft_Precise_Landing/Figures/generated"
os.makedirs(OUT, exist_ok=True)

m = sio.loadmat(DATA, squeeze_me=True, struct_as_record=False)
run = m["results"][3]   # IC4
d = run.data
N = int(d.idx) if int(d.idx) > 0 else d.tRange.shape[0]
t = d.tRange[:N]

p_1     = d.p_1[:, :N]      # 2 x N
E2_e    = d.V_s_e[:, :N]    # 2 x N (image-feature error used in outer funnel)
p_2     = d.p_2[:, :N]      # 3 x N
w_e     = d.V_h_e[:, :N]    # 3 x N (optic-flow tracking error)
sigma   = d.sigma[:, :N]    # 3 x N
kappa   = d.kappa[:, :N]    # 3 x N
kappa_a = d.kappa_a[:N]     # N
B_T_cd  = d.B_T_cd[:N]      # N (total thrust command, N)
I_a_cd  = d.I_a_cd[:, :N]   # 3 x N (desired inertial acceleration)
g_const = 9.81
m_uav   = float(d.m)
phi_max = np.deg2rad(35.0)
a_xy_limit = g_const * np.tan(phi_max)   # ~6.87 m/s^2 lateral cap from cone

T_max = float(d.T_max)
T_min = float(d.T_min)

# ---------- Plot A: outer visibility funnel ----------
fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.6), sharex=True)
labels = [r"$x$-axis", r"$y$-axis"]
for k, ax in enumerate(axes):
    ax.fill_between(t,  p_1[k], -p_1[k], color="orange", alpha=0.18,
                    label=r"$\pm\,p_1(t)$")
    ax.plot(t,  p_1[k], color="orange", lw=0.9)
    ax.plot(t, -p_1[k], color="orange", lw=0.9)
    ax.plot(t, E2_e[k], color="C0", label=r"$E_{2,k}(t)$")
    ax.set_xlabel(r"$t$ [s]")
    ax.set_ylabel("image error [px]")
    ax.set_title(labels[k])
axes[1].legend(loc="upper right")
fig.tight_layout()
fig.savefig(f"{OUT}/plasmc_outer_funnel.pdf")
plt.close(fig)

# ---------- Plot B: inner optic-flow funnel ----------
fig, axes = plt.subplots(1, 3, figsize=(7.0, 2.6), sharex=True)
labels = [r"$x$-axis", r"$y$-axis", r"$z$-axis"]
for k, ax in enumerate(axes):
    ax.fill_between(t,  p_2[k], -p_2[k], color="orange", alpha=0.18,
                    label=r"$\pm\,p_2(t)$")
    ax.plot(t,  p_2[k], color="orange", lw=0.9)
    ax.plot(t, -p_2[k], color="orange", lw=0.9)
    ax.plot(t, w_e[k], color="C0", label=r"$w_{e,k}(t)$")
    ax.set_xlabel(r"$t$ [s]")
    ax.set_title(labels[k])
axes[0].set_ylabel("optic-flow error")
axes[2].legend(loc="upper right")
fig.tight_layout()
fig.savefig(f"{OUT}/plasmc_inner_funnel.pdf")
plt.close(fig)

# ---------- Plot C: sliding surface ----------
fig, ax = plt.subplots(figsize=(7.0, 2.4))
for k, lbl in enumerate([r"$\sigma_x$", r"$\sigma_y$", r"$\sigma_z$"]):
    ax.plot(t, sigma[k], label=lbl)
ax.axhline(0.0, color="k", lw=0.6, ls=":")
ax.set_xlabel(r"$t$ [s]")
ax.set_ylabel(r"$\sigma(t)$")
ax.legend(loc="upper right", ncol=3)
fig.tight_layout()
fig.savefig(f"{OUT}/plasmc_sliding.pdf")
plt.close(fig)

# ---------- Plot D: adaptive gains ----------
fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.6))
for k, lbl in enumerate([r"$\kappa_x$", r"$\kappa_y$", r"$\kappa_z$"]):
    axes[0].plot(t, kappa[k], label=lbl)
axes[0].set_xlabel(r"$t$ [s]")
axes[0].set_ylabel(r"$\kappa(t)$")
axes[0].set_title("Translational ASMC gains")
axes[0].legend(loc="lower right", ncol=3)

axes[1].plot(t, kappa_a, color="C3", label=r"$\kappa_\mathrm{a}(t)$")
axes[1].set_xlabel(r"$t$ [s]")
axes[1].set_ylabel(r"$\kappa_\mathrm{a}(t)$")
axes[1].set_title("Yaw ASMC gain")
axes[1].legend(loc="lower right")
fig.tight_layout()
fig.savefig(f"{OUT}/plasmc_adaptive_gain.pdf")
plt.close(fig)

# ---------- Plot E: thrust + lateral accel ----------
a_xy_norm = np.linalg.norm(I_a_cd[:2], axis=0)
fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.6))
axes[0].plot(t, B_T_cd, color="C0", label=r"$T_\mathrm{cd}(t)$")
axes[0].axhline(T_max, color="r", lw=0.8, ls="--", label=r"$T_{\max}$")
axes[0].axhline(T_min, color="r", lw=0.8, ls="--")
axes[0].set_xlabel(r"$t$ [s]")
axes[0].set_ylabel("total thrust [N]")
axes[0].set_title("Thrust command")
axes[0].legend(loc="upper right")

axes[1].plot(t, a_xy_norm, color="C0", label=r"$\|^{\mathcal{I}}\!a_{\mathrm{cd},xy}\|$")
axes[1].axhline(a_xy_limit, color="r", lw=0.8, ls="--",
                label=r"$g\tan\phi_{\max}$")
axes[1].set_xlabel(r"$t$ [s]")
axes[1].set_ylabel(r"lateral accel [m/s$^2$]")
axes[1].set_title("Cone-constrained lateral acceleration")
axes[1].legend(loc="upper right")
fig.tight_layout()
fig.savefig(f"{OUT}/plasmc_thrust_accel.pdf")
plt.close(fig)

print("Wrote 5 PLASMC internals plots to:", OUT)
