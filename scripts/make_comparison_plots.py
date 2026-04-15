"""
Generate publication plots for the 5-controller comparison study.

Outputs PDFs into Figures/generated/:
  comparison_traj3d_sinusoidal.pdf — 5 controllers' UAV trajectories on the
                                      Sinusoidal target (IC4 = [2,2,-5])
  comparison_summary.pdf            — bar chart of landing time and final
                                      horizontal error across 5 trajectories
                                      x 5 controllers
"""
import os
import numpy as np
import scipy.io as sio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa

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
    "PLASMC (Proposed)": "PLASMC (Proposed)",
    "Lin 2022":          "Baseline A (PBVS--PPC)",
    "Zhang 2026":        "Baseline B (PBVS--AEDO)",
    "Chen 2025":         "Baseline C (IBVS--Obs)",
    "Cho 2022":          "Baseline D (FF--IBVS)",
}

# ---------- Plot F: 3D trajectories on Circular (deck) target ----------
m = sio.loadmat(f"{COMP}/Circular_comparison.mat",
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
    ax.scatter(X[0, -1], X[1, -1], -X[2, -1], color=color, marker="x", s=30)
    if not target_drawn:
        xt = d.x_t[:, :N]
        ax.plot(xt[0], xt[1], -xt[2], color="k", lw=0.8, ls="--",
                label="Target")
        target_drawn = True

ax.set_xlabel(r"$\,^\mathcal{I}x$ [m]")
ax.set_ylabel(r"$\,^\mathcal{I}y$ [m]")
ax.set_zlabel("altitude [m]")
ax.set_title("Circular ship-deck target (IC4 $=[2,2,-5]$~m)")
ax.legend(loc="upper left", bbox_to_anchor=(0.0, 0.95), ncol=2, fontsize=7)
ax.view_init(elev=22, azim=-58)
fig.tight_layout()
fig.savefig(f"{OUT}/comparison_traj3d_circular.pdf")
plt.close(fig)

# ---------- Plot G: bar chart summary (aggregated from .mat files) ----------
trajs = ["Static", "Linear", "Sinusoidal", "Circular", "Lissajous"]
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
    ax.set_xticklabels(trajs, rotation=20)
    ax.grid(axis="y", alpha=0.3)

axes[0].set_ylabel("landing time [s]")
axes[0].axhline(40.0, color="k", lw=0.8, ls=":")
axes[0].text(4.4, 40.4, r"$40\,$s budget", fontsize=7, ha="right")
axes[0].set_title("Landing time")

axes[1].set_ylabel("final $xy$ error [m]")
axes[1].axhline(0.05, color="k", lw=0.8, ls=":")
axes[1].text(4.4, 0.06, r"precise $0.05\,$m", fontsize=7, ha="right")
axes[1].set_title("Horizontal precision")

axes[2].set_ylabel(r"$\|v_\mathrm{rel}\|$ at touchdown [m/s]")
axes[2].axhline(0.20, color="k", lw=0.8, ls=":")
axes[2].text(4.4, 0.21, r"soft $0.20\,$m/s", fontsize=7, ha="right")
axes[2].set_title("Touchdown softness")

axes[0].legend(loc="upper left", fontsize=7, ncol=1)
fig.tight_layout()
fig.savefig(f"{OUT}/comparison_summary.pdf")
plt.close(fig)

print("Wrote comparison plots to:", OUT)
