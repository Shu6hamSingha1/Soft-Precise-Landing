"""
Generate publication plots for the 5-controller comparison study.

Outputs PDFs into Figures/generated/:
  comparison_traj3d_<traj>.pdf      — 5 controllers' UAV trajectories on each
                                      target trajectory (Static..Circular)
  comparison_summary.pdf            — 1x3 bar chart of landing time, final
                                      horizontal error, and touchdown softness
  comparison_combined_circular.pdf  — 1x4: Circular 3D + 3 summary bar charts,
                                      common bottom-center legend (main paper)
"""
import os
import numpy as np
import scipy.io as sio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

plt.rcParams.update({
    "font.family": "serif",
    "font.size": 9,
    "axes.labelsize": 9,
    "axes.titlesize": 10,
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
})

from pathlib import Path
ROOT = str(Path(__file__).resolve().parent.parent)
COMP = f"{ROOT}/MATLAB/Comparison/Datasets"
OUT  = f"{ROOT}/Soft_Precise_Landing/Figures/generated"
os.makedirs(OUT, exist_ok=True)

CTRL_COLORS = {
    "PLASMC (Proposed)": "C3",
    "Lin 2022":          "C0",
    "Zhang 2026":        "C2",
    "Chen 2025":         "C4",
    "Cho 2022":          "C1",
}

# Display labels for figure legends (no author names; matches manuscript's
# author-less citation style). The dict keys above still match the MATLAB
# ctrl_name strings; only the rendered labels change.
CTRL_DISPLAY = {
    "PLASMC (Proposed)": "MDF-ASMC (Proposed)",
    "Lin 2022":          "Baseline A (PBVS--PPC)",
    "Zhang 2026":        "Baseline B (PBVS--AEDO)",
    "Chen 2025":         "Baseline C (IBVS--Obs)",
    "Cho 2022":          "Baseline D (FF--IBVS)",
}

# ---------- Plot F: 3D trajectories on Circular (deck) target ----------
PRECISE_XY_M     = 0.08    # precise-landing horizontal threshold
SOFT_VZ_REL_MPS  = 0.20    # soft-landing relative vertical-speed threshold
Z_F_M            = 0.20    # above-target gap at termination (corridor vertical height)


def draw_landing_corridor(ax, xt, yt, zt, half_xy=PRECISE_XY_M,
                          z_height=Z_F_M, color="0.6", alpha=0.18,
                          edge_color="k", edge_lw=0.6, edge_ls="--",
                          label=None):
    """3D corridor around the target trajectory representing the soft-precise
    allowable landing region: xy ±half_xy perpendicular to tangent; vertical
    from true target altitude up to (target altitude + z_height), following
    target heave. Inputs in NED."""
    xt = np.asarray(xt); yt = np.asarray(yt); zt = np.asarray(zt)
    zB = -zt; zT = -zt + z_height
    n = len(xt)
    if n < 2:
        return
    dx = np.gradient(xt); dy = np.gradient(yt)
    mag = np.hypot(dx, dy)
    if np.max(mag) < 1e-3:
        sx = np.array([-1, 1, 1, -1]) * half_xy + xt[0]
        sy = np.array([-1, -1, 1, 1]) * half_xy + yt[0]
        zT0 = zT[0]; zB0 = zB[0]
        top    = list(zip(sx, sy, np.full(4, zT0)))
        bottom = list(zip(sx, sy, np.full(4, zB0)))
        sides = [[top[i], top[(i+1) % 4], bottom[(i+1) % 4], bottom[i]] for i in range(4)]
        ax.add_collection3d(Poly3DCollection([top, bottom] + sides,
                                             facecolor=color, alpha=alpha,
                                             edgecolor="none"))
        ax.plot([*sx, sx[0]], [*sy, sy[0]],
                np.full(5, zT0), color=edge_color, lw=edge_lw, ls=edge_ls,
                label=label)
        return
    safe = np.where(mag < 1e-9, 1.0, mag)
    nx = -dy / safe; ny = dx / safe
    xL = xt - half_xy * nx; yL = yt - half_xy * ny
    xR = xt + half_xy * nx; yR = yt + half_xy * ny
    LT = list(zip(xL, yL, zT)); RT = list(zip(xR, yR, zT))
    LB = list(zip(xL, yL, zB)); RB = list(zip(xR, yR, zB))
    def strip(P, Q):
        return [[P[i], Q[i], Q[i+1], P[i+1]] for i in range(n - 1)]
    faces = strip(LT, RT) + strip(LB, RB) + strip(LT, LB) + strip(RT, RB)
    ax.add_collection3d(Poly3DCollection(faces, facecolor=color, alpha=alpha,
                                         edgecolor="none"))
    ax.plot(xL, yL, zT, color=edge_color, lw=edge_lw, ls=edge_ls, label=label)
    ax.plot(xR, yR, zT, color=edge_color, lw=edge_lw, ls=edge_ls)
    ax.plot(xL, yL, zB, color=edge_color, lw=edge_lw, ls=edge_ls)
    ax.plot(xR, yR, zB, color=edge_color, lw=edge_lw, ls=edge_ls)

_TRAJ_TITLE = {
    "Static":     ("Case 1", "Static Target"),
    "Linear":     ("Case 2", "Linear Target Trajectory"),
    "Sinusoidal": ("Case 3", "Sinusoidal Target Trajectory"),
    "Lissajous":  ("Case 4", "Lissajous Target Trajectory"),
    "Circular":   ("Case 5", "Circular Target Trajectory"),
}

for traj, (case, title_traj) in _TRAJ_TITLE.items():
    m = sio.loadmat(f"{COMP}/{traj}_comparison.mat",
                    squeeze_me=True, struct_as_record=False)
    fig = plt.figure(figsize=(7.0, 4.6))
    ax = fig.add_subplot(111, projection="3d")

    target_drawn = False
    for run in m["all_results"]:
        d = run.data
        N = int(d.idx) if int(d.idx) > 0 else d.X_DS.shape[1]
        X = d.X_DS[:, :N]
        name = str(run.ctrl_name)
        color = CTRL_COLORS.get(name, "k")
        disp  = CTRL_DISPLAY.get(name, name)
        ax.plot(X[0], X[1], -X[2], color=color, lw=1.3, label=disp)
        ax.scatter(X[0, 0], X[1, 0], -X[2, 0], color=color, marker="o", s=20)
        tgt_xyz  = d.x_t[:3, :N]
        dtgt_xyz = d.dx_t[:3, :N] if hasattr(d, "dx_t") else np.zeros_like(tgt_xyz)
        xy_err = float(np.linalg.norm(X[:2, -1] - tgt_xyz[:2, -1]))
        vz_rel = float(abs(X[9, -1] - dtgt_xyz[2, -1]))
        soft_precise = (xy_err <= PRECISE_XY_M) and (vz_rel <= SOFT_VZ_REL_MPS)
        end_marker = "^" if soft_precise else "x"
        ax.scatter(X[0, -1], X[1, -1], -X[2, -1], color=color,
                   marker=end_marker, s=30)
        if not target_drawn:
            xt = d.x_t[:, :N]
            draw_landing_corridor(ax, xt[0], xt[1], xt[2],
                                  label="Target corridor")
            target_drawn = True

    ax.set_xlabel(r"$\,^\mathcal{I}x$ [m]", labelpad=2)
    ax.set_ylabel(r"$\,^\mathcal{I}y$ [m]", labelpad=2)
    ax.set_zlabel("altitude [m]", labelpad=2)
    ax.locator_params(axis="x", nbins=4)
    ax.locator_params(axis="y", nbins=4)
    ax.locator_params(axis="z", nbins=4)
    ax.tick_params(pad=1, labelsize=7)
    ax.set_title(f"{case}: {title_traj} (IC$_2=[2,2,-5]$~m)", fontsize=8, y=0.95)
    # Vertical legend just past the z-label, sized to stay inside figure.
    ax.legend(loc="center left", bbox_to_anchor=(1.10, 0.5),
              ncol=1, fontsize=6)
    ax.view_init(elev=22, azim=-58)
    fig.tight_layout()
    fig.subplots_adjust(right=0.72)
    fig.savefig(f"{OUT}/comparison_traj3d_{traj.lower()}.pdf")
    plt.close(fig)

# ---------- Plot G: bar chart summary (aggregated from .mat files) ----------
trajs = ["Static", "Linear", "Sinusoidal", "Lissajous", "Circular"]
traj_labels = ["Case 1", "Case 2", "Case 3", "Case 4", "Case 5"]
ctrls = ["PLASMC (Proposed)", "Lin 2022", "Zhang 2026", "Chen 2025", "Cho 2022"]

Z_LAND = 0.205

def aggregate(traj):
    mat = sio.loadmat(f"{COMP}/{traj}_comparison.mat",
                      squeeze_me=True, struct_as_record=False)
    names = [str(x) for x in np.atleast_1d(mat["ctrl_names"])]
    runs  = np.atleast_1d(mat["all_results"])
    row_t, row_xy, row_v = {}, {}, {}
    for name, run in zip(names, runs):
        d = run.data
        N = int(d.idx) if int(d.idx) > 0 else d.tRange.shape[0]
        t   = d.tRange[:N]
        X   = d.X_DS[:, :N]
        alt = -X[2, :]
        tgt = d.x_t[:3, :N]
        dtgt= d.dx_t[:3, :N] if hasattr(d, "dx_t") else np.zeros_like(X[:3])
        xy_err = np.linalg.norm(X[:2] - tgt[:2], axis=0)
        vrel   = np.linalg.norm(X[7:10] - dtgt, axis=0)  # UAV vel at 8:10 (0-indexed 7:10)
        below = np.where(alt <= Z_LAND)[0]
        k = below[0] if len(below) else N - 1
        row_t[name]  = float(t[k])
        row_xy[name] = float(xy_err[k])
        row_v[name]  = float(vrel[k])
    return row_t, row_xy, row_v

LAND_TIME = np.zeros((len(trajs), len(ctrls)))
FINAL_XY  = np.zeros_like(LAND_TIME)
FINAL_V   = np.zeros_like(LAND_TIME)
for i, tr in enumerate(trajs):
    rt, rxy, rv = aggregate(tr)
    for j, name in enumerate(ctrls):
        LAND_TIME[i, j] = rt[name]
        FINAL_XY[i, j]  = rxy[name]
        FINAL_V[i, j]   = rv[name]

fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.2))
x = np.arange(len(trajs))
width = 0.16

for j, name in enumerate(ctrls):
    color = CTRL_COLORS[name]
    disp  = CTRL_DISPLAY[name]
    axes[0].bar(x + (j - 2) * width, LAND_TIME[:, j], width, color=color, label=disp)
    axes[1].bar(x + (j - 2) * width, FINAL_XY[:, j],  width, color=color, label=disp)
    axes[2].bar(x + (j - 2) * width, FINAL_V[:, j],   width, color=color, label=disp)

for ax in axes:
    ax.set_xticks(x)
    ax.set_xticklabels(traj_labels, rotation=20)
    ax.grid(axis="y", alpha=0.3)

axes[0].set_ylabel("landing time [s]")
axes[0].set_title("Landing time")

axes[1].set_ylabel("final $xy$ error [m]")
axes[1].set_title("Horizontal precision")

axes[2].set_ylabel(r"$\|v_\mathrm{rel}\|$ at touchdown [m/s]")
axes[2].set_title("Touchdown softness")

axes[0].get_legend().remove() if axes[0].get_legend() else None
# Shared legend at LEFT CENTER of the figure, vertical stack, outside subplots.
handles, labels = axes[0].get_legend_handles_labels()
fig.legend(handles, labels, loc="center left", ncol=1,
           bbox_to_anchor=(0.0, 0.5), frameon=False, fontsize=8)
fig.tight_layout(rect=(0.18, 0.0, 1.0, 1.0))
fig.savefig(f"{OUT}/comparison_summary.pdf")
plt.close(fig)

# ---------- Plot H: Combined 1x4 (3D Circular + 3 summary bar charts) ----------
# Main-paper Fig.: first subplot is the Circular 3D trajectory; subplots 2-4
# are the bar charts replicated from comparison_summary.pdf. Common legend at
# bottom-center, horizontal stacking. Format mirrors multi_init/<Traj>_combined.pdf
# (figsize ~3:1 of multi_init's 1x2; suptitle 24, subplot titles 20, axis
# labels 20, ticks 18, legend 14). Subplot region pinned via bottom=0.230,
# top=0.865 so all four boxes share identical aspect.
fig = plt.figure(figsize=(18.0, 5.0))
gs  = fig.add_gridspec(1, 4, left=0.05, right=0.99, wspace=0.40)
ax3d  = fig.add_subplot(gs[0, 0], projection="3d")
ax_t  = fig.add_subplot(gs[0, 1])
ax_xy = fig.add_subplot(gs[0, 2])
ax_v  = fig.add_subplot(gs[0, 3])

# 3D Circular subplot
m_circ = sio.loadmat(f"{COMP}/Circular_comparison.mat",
                     squeeze_me=True, struct_as_record=False)
target_drawn = False
for run in m_circ["all_results"]:
    d = run.data
    N = int(d.idx) if int(d.idx) > 0 else d.X_DS.shape[1]
    X = d.X_DS[:, :N]
    name  = str(run.ctrl_name)
    color = CTRL_COLORS.get(name, "k")
    disp  = CTRL_DISPLAY.get(name, name)
    ax3d.plot(X[0], X[1], -X[2], color=color, lw=1.3, label=disp)
    ax3d.scatter(X[0, 0], X[1, 0], -X[2, 0], color=color, marker="o", s=20)
    tgt_xyz  = d.x_t[:3, :N]
    dtgt_xyz = d.dx_t[:3, :N] if hasattr(d, "dx_t") else np.zeros_like(tgt_xyz)
    xy_err = float(np.linalg.norm(X[:2, -1] - tgt_xyz[:2, -1]))
    vz_rel = float(abs(X[9, -1] - dtgt_xyz[2, -1]))
    soft_precise = (xy_err <= PRECISE_XY_M) and (vz_rel <= SOFT_VZ_REL_MPS)
    end_marker = "^" if soft_precise else "x"
    ax3d.scatter(X[0, -1], X[1, -1], -X[2, -1], color=color,
                 marker=end_marker, s=30)
    if not target_drawn:
        xt = d.x_t[:, :N]
        draw_landing_corridor(ax3d, xt[0], xt[1], xt[2])
        target_drawn = True

ax3d.set_xlabel(r"$\,^\mathcal{I}x$ [m]", labelpad=12, fontsize=20)
ax3d.set_ylabel(r"$\,^\mathcal{I}y$ [m]", labelpad=12, fontsize=20)
ax3d.set_zlabel("altitude [m]", labelpad=2, fontsize=20)
ax3d.locator_params(axis="x", nbins=4)
ax3d.locator_params(axis="y", nbins=4)
ax3d.locator_params(axis="z", nbins=4)
ax3d.tick_params(pad=1, labelsize=14)
ax3d.set_title("Landing Performance for Case 5", fontsize=20, x=0.55, y=0.95)
ax3d.view_init(elev=22, azim=-58)

# Bar charts: reuse LAND_TIME / FINAL_XY / FINAL_V already computed above.
xb = np.arange(len(trajs))
width = 0.16
for j, name in enumerate(ctrls):
    color = CTRL_COLORS[name]
    disp  = CTRL_DISPLAY[name]
    ax_t.bar(xb + (j - 2) * width, LAND_TIME[:, j], width, color=color, label=disp)
    ax_xy.bar(xb + (j - 2) * width, FINAL_XY[:, j], width, color=color)
    ax_v.bar(xb + (j - 2) * width, FINAL_V[:, j],  width, color=color)

for ax in (ax_t, ax_xy, ax_v):
    ax.set_xticks(xb)
    ax.set_xticklabels(traj_labels, rotation=20, fontsize=16)
    ax.tick_params(axis="y", labelsize=16)
    ax.grid(axis="y", alpha=0.3)

ax_t.set_ylabel("landing time [s]", fontsize=20, labelpad=4)
ax_t.set_title("Landing Time", fontsize=20, y=1.03)
ax_xy.set_ylabel("final $xy$ error [m]", fontsize=20, labelpad=4)
ax_xy.set_title("Horizontal Precision", fontsize=20, y=1.03)
ax_v.set_ylabel(r"$\|v_\mathrm{rel}\|$ at touchdown [m/s]", fontsize=20, labelpad=4)
ax_v.set_title("Touchdown Softness", fontsize=20, y=1.03)

# Common legend at bottom-center, horizontal (5 entries -> ncol=5).
handles, labels = ax_t.get_legend_handles_labels()
fig.legend(handles, labels, loc="lower center", ncol=5,
           bbox_to_anchor=(0.5, 0.0), frameon=False, fontsize=14,
           handlelength=1.6, columnspacing=2.0, handletextpad=0.6)

fig.suptitle("Landing Performance of Five Controllers across Cases 1–5",
             fontsize=24, y=0.99)

# Establish a baseline subplot region (matches multi_init's vertical extent),
# then manually re-position each subplot: 3-D panel (subplot 1) is enlarged
# and shifted to the figure's left edge; the three bar-chart panels are
# shortened vertically and re-distributed across the remaining width so they
# sit centred against the (taller) 3-D panel.
fig.subplots_adjust(bottom=0.230, top=0.865)

# 3-D panel: bbox sized close to the cube's natural display aspect so there
# is no internal horizontal padding flanking the 3-D content. Fully on the
# figure's left edge.
ax3d.set_position([0.000, 0.10, 0.24, 0.84])

# Bar charts: shorter (height 0.50) and vertically centred against the 3-D box.
# Generous horizontal gap so the next chart's y-axis label clears the previous
# chart's rotated x-tick labels (Case 5 sticks out farthest right).
bar_y0, bar_h = 0.23, 0.56
bar_left, bar_right = 0.33, 0.99
n_bars = 3
gap = 0.07
bar_w = (bar_right - bar_left - (n_bars - 1) * gap) / n_bars
for k, ax in enumerate((ax_t, ax_xy, ax_v)):
    x0 = bar_left + k * (bar_w + gap)
    ax.set_position([x0, bar_y0, bar_w, bar_h])

# bbox_inches="tight" silently drops 3-D z-axis labels, so plain savefig.
fig.savefig(f"{OUT}/comparison_combined_circular.pdf")
plt.close(fig)

print("Wrote comparison plots to:", OUT)
