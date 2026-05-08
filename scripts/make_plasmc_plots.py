"""
Generate publication-quality MDF-ASMC internals plots from Multi_init_cond datasets.

Outputs PDFs into Figures/generated/:
  plasmc_funnel_combined.pdf — outer visibility funnel + inner optic-flow funnel
                               (1x4 figure*; supersedes the standalone outer/inner
                               plots which were moved to Obsolete/ on 2026-05-07).
  plasmc_sliding.pdf         — sliding surface sigma per axis
  plasmc_adaptive_gain.pdf   — kappa(t) and kappa_a(t)
  plasmc_thrust_accel.pdf    — total thrust + lateral acceleration vs cone bound

The standalone plasmc_outer_funnel.pdf and plasmc_inner_funnel.pdf code blocks
are retained under `if False:` for easy revival.

Representative case: IC2 = [2,2,-5] on Sinusoidal trajectory (index 1, 0-based).
"""
import os
import numpy as np
import scipy.io as sio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3D projection)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["cmr10", "Computer Modern Roman", "DejaVu Serif"],
    "mathtext.fontset": "cm",
    "axes.formatter.use_mathtext": True,
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
run = m["results"][1]   # IC2 = [2,2,-5]
d = run.data
N = int(d.idx) if int(d.idx) > 0 else d.tRange.shape[0]
t = d.tRange[:N]

# Approach 2 (2026-04-18): outer p_1 funnel replaced by a funnel-margin
# rectangular box on the four physical corner pixels. The two performance
# functions are rho_fov_x(t), rho_fov_y(t) (exponentially decreasing).
rho_fov = d.rho_fov_log[:, :N]   # 2 x N (px)
corners = d.P_DS[:, 0:4, :N]     # 2 x 4 x N — physical corner pixel coords

# Desired feature points (constant in pixel coordinates), per
# MATLAB/Multi_init_cond/InitVar.m: V_nP_d = (f/(2*zf)) * V_nP3(1:2,:),
# evaluated at init (V_nP3 = T_nP3 at t=0).
V_nP_d = d.V_nP_d                # 2 x 4 (px), constant
p_2     = d.p_2[:, :N]      # 3 x N
h_e     = d.V_h_e[:, :N]    # 3 x N (optic-flow tracking error)
sigma   = d.sigma[:, :N]    # 3 x N
kappa   = d.kappa[:, :N]    # 3 x N
kappa_a = d.kappa_a[:N]     # N
B_T_cd  = d.B_T_cd[:N]      # N (total thrust command, N)
I_a_cd  = d.I_a_cd[:, :N]   # 3 x N (desired inertial acceleration)
g_const = 9.81
m_uav   = float(d.m)
theta_cap = np.deg2rad(60.0)             # locked Property 1 value
a_xy_limit = g_const * np.tan(theta_cap) # lateral cap from cone clamp

T_max = float(d.T_max)
T_min = float(d.T_min)

# Shared variables consumed by Plot B+ (combined funnel) below.
# Originally defined inside Plots A and B; hoisted here when those two
# standalone plots were commented out 2026-05-07.
t_end = float(t[-1])
sample_t = [0.0] + list(np.arange(3.0, t_end, 3.0)) + [t_end]
# Corner colours: C4 (purple) replaces C1 (orange) so corners are visually
# distinct from the orange funnel envelope.
corner_colors = ["C0", "C4", "C2", "C3"]
n_corners = V_nP_d.shape[1]
t0_f, tF_f = float(t[0]), float(t[-1])

# OBSOLETE 2026-05-07: standalone plasmc_outer_funnel.pdf (single 3-D plot)
# is superseded by plasmc_funnel_combined.pdf which carries the same outer
# funnel as col 1 of a 1x4 figure*. PDF moved to
# Obsolete/Soft_Precise_Landing/Figures/generated/. Block kept under
# `if False:` for easy revival.
if False:
    # ---------- Plot A: 3-D visibility funnel + 4 corner trajectories ----------
    # Funnel walls are the four edges of the rectangle [-rho_x, +rho_x] x
    # [-rho_y, +rho_y] traced over t. Corner pixel trajectories live inside.
    # Sampled quadrilaterals (start, every 3 s, end) connect the four corners.
    fig = plt.figure(figsize=(5.0, 4.5))
    ax = fig.add_subplot(111, projection="3d")

    edge_signs = [(+1, +1), (+1, -1), (-1, +1), (-1, -1)]
    for idx_e, (su, sv) in enumerate(edge_signs):
        ax.plot(t, sv * rho_fov[1], su * rho_fov[0],
                color="orange", lw=0.9, alpha=0.85,
                label=r"$\boldsymbol{p}_1(t)$" if idx_e == 0 else None)

    for ts in sample_t:
        ti = int(np.argmin(np.abs(t - ts)))
        rx, ry = float(rho_fov[0, ti]), float(rho_fov[1, ti])
        bx = np.array([-rx, +rx, +rx, -rx, -rx])
        by = np.array([-ry, -ry, +ry, +ry, -ry])
        bt = np.full(5, t[ti])
        ax.plot(bt, by, bx, color="orange", lw=0.8, ls="--", alpha=0.6)
        cx = np.append(corners[0, :, ti], corners[0, 0, ti])
        cy = np.append(corners[1, :, ti], corners[1, 0, ti])
        ct = np.full(5, t[ti])
        ax.plot(ct, cy, cx, color="C0", lw=1.0, alpha=0.9)

    for k in range(4):
        ax.plot(t, corners[1, k], corners[0, k],
                color=corner_colors[k], lw=0.9,
                label=fr"$\,^\mathcal{{C}}\hat{{\boldsymbol{{r}}}}_{k+1}$")

    t_full = np.array([t[0], t[-1]])
    for k in range(4):
        ax.plot(t_full, [V_nP_d[1, k]] * 2, [V_nP_d[0, k]] * 2,
                color="k", lw=0.8, ls="--", alpha=0.85)

    walls = []
    for i in range(n_corners):
        j = (i + 1) % n_corners
        walls.append([
            (t0_f, V_nP_d[1, i], V_nP_d[0, i]),
            (t0_f, V_nP_d[1, j], V_nP_d[0, j]),
            (tF_f, V_nP_d[1, j], V_nP_d[0, j]),
            (tF_f, V_nP_d[1, i], V_nP_d[0, i]),
        ])
    ax.add_collection3d(Poly3DCollection(
        walls, facecolor="0.3", alpha=0.08, edgecolor="none"))

    dx = np.append(V_nP_d[0, :], V_nP_d[0, 0])
    dy = np.append(V_nP_d[1, :], V_nP_d[1, 0])
    for ts, lbl in [(t[0], r"$\hat{\boldsymbol{r}}_{i,\text{d}}$"),
                    (t[-1], None)]:
        dt = np.full(5, ts)
        ax.plot(dt, dy, dx, color="k", lw=0.9, ls="--", alpha=0.85, label=lbl)

    ax.set_xlabel(r"$t$ [s]")
    ax.set_ylabel(r"$\,^\mathcal{C}\hat{y}$ [px]")
    ax.set_zlabel(r"$\,^\mathcal{C}\hat{x}$ [px]")
    ax.set_title("Target Image Funnel and Feature Point Trajectories")
    ax.legend(loc="upper left", fontsize=7, ncol=2)
    ax.view_init(elev=18, azim=-60)
    fig.tight_layout()
    fig.savefig(f"{OUT}/plasmc_outer_funnel.pdf")
    plt.close(fig)

# OBSOLETE 2026-05-07: standalone plasmc_inner_funnel.pdf (1x3 grid) is
# superseded by plasmc_funnel_combined.pdf which carries the same inner
# funnel as cols 2-4 of a 1x4 figure*. PDF moved to
# Obsolete/Soft_Precise_Landing/Figures/generated/. Block kept under
# `if False:` for easy revival.
if False:
    # ---------- Plot B: inner optic-flow funnel ----------
    fig, axes = plt.subplots(1, 3, figsize=(7.0, 2.6), sharex=True)
    labels = [r"$x$-axis", r"$y$-axis", r"$z$-axis"]
    for k, ax in enumerate(axes):
        ax.fill_between(t,  p_2[k], -p_2[k], color="orange", alpha=0.18,
                        label=r"$\pm\boldsymbol{p}_2(t)$")
        ax.plot(t,  p_2[k], color="orange", lw=0.9)
        ax.plot(t, -p_2[k], color="orange", lw=0.9)
        ax.plot(t, h_e[k], color="C0", label=r"$\boldsymbol{h}_{\text{e},k}(t)$")
        ax.set_xlabel(r"$t$ [s]")
        ax.set_title(labels[k])
    axes[0].set_ylabel("optic-flow error")
    axes[2].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(f"{OUT}/plasmc_inner_funnel.pdf")
    plt.close(fig)

# ---------- Plot B+: combined outer + inner funnel (1x4 figure*) ----------
# 1x4 layout mirroring comparison_combined_circular.pdf:
#   col 1 = outer 3-D funnel (target image funnel + corner trajectories)
#   cols 2-4 = inner optic-flow funnel per axis
# Common bottom-center legend.
fig = plt.figure(figsize=(18.0, 5.0))
gs  = fig.add_gridspec(1, 4, left=0.05, right=0.99, wspace=0.40)
ax3d = fig.add_subplot(gs[0, 0], projection="3d")
ax_x = fig.add_subplot(gs[0, 1])
ax_y = fig.add_subplot(gs[0, 2])
ax_z = fig.add_subplot(gs[0, 3])

# --- col 1: outer 3-D funnel ---
edge_signs = [(+1, +1), (+1, -1), (-1, +1), (-1, -1)]
edges_3d = {}
for idx_e, (su, sv) in enumerate(edge_signs):
    pts = np.column_stack([t, sv * rho_fov[1], su * rho_fov[0]])
    edges_3d[(su, sv)] = pts
    ax3d.plot(pts[:, 0], pts[:, 1], pts[:, 2],
              color="orange", lw=0.9, alpha=0.85,
              label=r"$\boldsymbol{p}_1(t)$" if idx_e == 0 else None)

# Sampled funnel rectangles (orange dashed) and actual quadrilaterals (blue).
for ts in sample_t:
    ti = int(np.argmin(np.abs(t - ts)))
    rx, ry = float(rho_fov[0, ti]), float(rho_fov[1, ti])
    bx = np.array([-rx, +rx, +rx, -rx, -rx])
    by = np.array([-ry, -ry, +ry, +ry, -ry])
    bt = np.full(5, t[ti])
    ax3d.plot(bt, by, bx, color="orange", lw=0.8, ls="--", alpha=0.6)
    cx = np.append(corners[0, :, ti], corners[0, 0, ti])
    cy = np.append(corners[1, :, ti], corners[1, 0, ti])
    ct = np.full(5, t[ti])
    ax3d.plot(ct, cy, cx, color="C0", lw=1.0, alpha=0.9)

# 4 corner trajectories.
for k in range(4):
    ax3d.plot(t, corners[1, k], corners[0, k],
              color=corner_colors[k], lw=0.9,
              label=fr"$\,^\mathcal{{C}}\hat{{\boldsymbol{{r}}}}_{k+1}$")

# Black dashed reference lines for desired corners.
t_full2 = np.array([t[0], t[-1]])
for k in range(4):
    ax3d.plot(t_full2, [V_nP_d[1, k]] * 2, [V_nP_d[0, k]] * 2,
              color="k", lw=0.8, ls="--", alpha=0.85)

# Filled desired tunnel.
walls2 = []
for i in range(n_corners):
    j = (i + 1) % n_corners
    walls2.append([
        (t0_f, V_nP_d[1, i], V_nP_d[0, i]),
        (t0_f, V_nP_d[1, j], V_nP_d[0, j]),
        (tF_f, V_nP_d[1, j], V_nP_d[0, j]),
        (tF_f, V_nP_d[1, i], V_nP_d[0, i]),
    ])
ax3d.add_collection3d(Poly3DCollection(
    walls2, facecolor="0.3", alpha=0.08, edgecolor="none"))

# Desired quadrilaterals at start AND end (dashed black).
dx2 = np.append(V_nP_d[0, :], V_nP_d[0, 0])
dy2 = np.append(V_nP_d[1, :], V_nP_d[1, 0])
for ts2, lbl2 in [(t[0], r"$\hat{\boldsymbol{r}}_{i,\text{d}}$"),
                  (t[-1], None)]:
    dt2 = np.full(5, ts2)
    ax3d.plot(dt2, dy2, dx2, color="k", lw=0.9, ls="--", alpha=0.85, label=lbl2)

ax3d.set_xlabel(r"$t$ [s]", labelpad=12, fontsize=20)
ax3d.set_ylabel(r"$\,^\mathcal{C}\hat{y}$ [px]", labelpad=12, fontsize=20)
ax3d.set_zlabel(r"$\,^\mathcal{C}\hat{x}$ [px]", labelpad=2, fontsize=20)
ax3d.locator_params(axis="x", nbins=4)
ax3d.locator_params(axis="y", nbins=4)
ax3d.locator_params(axis="z", nbins=4)
ax3d.tick_params(pad=1, labelsize=14)
ax3d.set_title("Target Image Funnel", fontsize=20, x=0.55, y=0.95)
ax3d.view_init(elev=18, azim=-60)

# --- cols 2-4: inner optic-flow funnel per axis ---
inner_axes = (ax_x, ax_y, ax_z)
inner_ylabels = [r"$h_{\text{e},x}$ [1/s]",
                 r"$h_{\text{e},y}$ [1/s]",
                 r"$h_{\text{e},z}$ [1/s]"]
for k, ax in enumerate(inner_axes):
    ax.fill_between(t, p_2[k], -p_2[k], color="orange", alpha=0.18,
                    label=r"$\pm\boldsymbol{p}_2(t)$" if k == 0 else None)
    ax.plot(t,  p_2[k], color="orange", lw=0.9)
    ax.plot(t, -p_2[k], color="orange", lw=0.9)
    ax.plot(t, h_e[k], color="C0", lw=1.2,
            label=r"$\boldsymbol{h}_{\text{e},k}(t)$" if k == 0 else None)
    ax.set_xlabel(r"$t$ [s]", fontsize=20, labelpad=4)
    ax.set_ylabel(inner_ylabels[k], fontsize=20, labelpad=4)
    ax.tick_params(axis="both", labelsize=16)
    ax.grid(axis="y", alpha=0.3)

# Separate legends per panel: outer 3-D in its own subplot, inner-funnel
# legend on the rightmost (z-axis) subplot.
ax3d.legend(loc="upper center", bbox_to_anchor=(0.75, 0.85),
            fontsize=12, ncol=2,
            handlelength=1.4, columnspacing=1.2, handletextpad=0.5)
hin, lin = ax_x.get_legend_handles_labels()
ax_z.legend(hin, lin, loc="upper right", fontsize=14,
            handlelength=1.4, handletextpad=0.5)

fig.suptitle("Dual-Funnel Invariance under MDF-ASMC", fontsize=24, y=0.99)

# Manual layout: 3-D panel hugs left edge, 3 inner panels distributed across
# the remaining width.
fig.subplots_adjust(bottom=0.140, top=0.865)
ax3d.set_position([0.000, 0.02, 0.24, 0.92])
inner_y0, inner_h = 0.16, 0.65
inner_left, inner_right = 0.33, 0.99
n_inner = 3
gap = 0.07
inner_w = (inner_right - inner_left - (n_inner - 1) * gap) / n_inner
for k, ax in enumerate(inner_axes):
    x0 = inner_left + k * (inner_w + gap)
    ax.set_position([x0, inner_y0, inner_w, inner_h])

# Shared subtitle "Optic Flow Funnel" spanning subplots 2-4.
inner_center_x = (inner_left + inner_right) / 2
fig.text(inner_center_x, inner_y0 + inner_h + 0.02,
         "Optic Flow Funnel", ha="center", va="bottom", fontsize=20)

fig.savefig(f"{OUT}/plasmc_funnel_combined.pdf")
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
axes[0].set_title("Translational ASMC Gains")
axes[0].legend(loc="lower right", ncol=3)

axes[1].plot(t, kappa_a, color="C3", label=r"$\kappa_\mathrm{a}(t)$")
axes[1].set_xlabel(r"$t$ [s]")
axes[1].set_ylabel(r"$\kappa_\mathrm{a}(t)$")
axes[1].set_title("Yaw ASMC Gain")
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
axes[0].set_title("Thrust Command")
axes[0].legend(loc="upper right")

axes[1].plot(t, a_xy_norm, color="C0", label=r"$\|^{\mathcal{I}}\!a_{\mathrm{cd},xy}\|$")
axes[1].axhline(a_xy_limit, color="r", lw=0.8, ls="--",
                label=r"$g\tan\theta_{\mathrm{cap}}$")
axes[1].set_xlabel(r"$t$ [s]")
axes[1].set_ylabel(r"lateral accel [m/s$^2$]")
axes[1].set_title("Cone-Constrained Lateral Acceleration")
axes[1].legend(loc="upper right")
fig.tight_layout()
fig.savefig(f"{OUT}/plasmc_thrust_accel.pdf")
plt.close(fig)

print("Wrote 4 MDF-ASMC internals plots to:", OUT)
