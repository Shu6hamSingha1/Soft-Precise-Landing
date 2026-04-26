"""
Per-trajectory multi-speed comparison plots for the four baseline controllers.

Mirrors the MDF-ASMC plot in plasmc_multi_speed_landing.pdf, but per-baseline
and per-trajectory: one PDF per moving trajectory, with a 2x2 grid of
subplots (Lin 2022, Zhang 2026, Chen 2025, Cho 2022). Each subplot overlays
the controller's UAV descent at five speed multipliers lambda in {0.6, 0.8,
1.0, 1.2, 1.4}, with per-lambda soft-precise allowable landing corridor and
triangle/cross touchdown markers.

Reads:
  MATLAB/Comparison/Datasets/<traj>_multi_speed_comparison.mat
  (produced by MATLAB/Comparison/multi_speed_comparison.m)

Outputs:
  Soft_Precise_Landing/Figures/generated/comparison_multi_speed_<traj>.pdf
  for traj in {linear, sinusoidal, lissajous, circular}.
"""
import os
import numpy as np
import scipy.io as sio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.cm import viridis
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
    "lines.linewidth": 1.2,
})

from pathlib import Path
ROOT     = str(Path(__file__).resolve().parent.parent)
DATA_DIR = f"{ROOT}/MATLAB/Comparison/Datasets"
OUT_DIR  = f"{ROOT}/Soft_Precise_Landing/Figures/generated"
os.makedirs(OUT_DIR, exist_ok=True)

TRAJS = ["Linear", "Sinusoidal", "Lissajous", "Circular"]
MULTS = [0.6, 0.8, 1.0, 1.2, 1.4]

PRECISE_XY_M    = 0.08
SOFT_VZ_REL_MPS = 0.20
Z_F_M           = 0.20

CTRL_ORDER  = [2, 3, 4, 5]
CTRL_TITLE  = {2: "Baseline A (PBVS--PPC)",
               3: "Baseline B (PBVS--AEDO)",
               4: "Baseline C (IBVS--Obs)",
               5: "Baseline D (FF--IBVS)"}

CASE_OF = {"Linear": "Case 2", "Sinusoidal": "Case 3",
           "Lissajous": "Case 4", "Circular": "Case 5"}
VEL_EQN = {
    "Linear":     r"$\,^\mathcal{I}\boldsymbol{v}_\text{t} = [\lambda,\,\lambda,\,0.1\cos(0.5\,t)]^\top$ m/s",
    "Sinusoidal": r"$\,^\mathcal{I}\boldsymbol{v}_\text{t} = [0.4\lambda\cos(0.8\lambda t),\,0.3\lambda,\,0]^\top$ m/s",
    "Lissajous":  r"$\,^\mathcal{I}\boldsymbol{v}_\text{t} = [-0.32\lambda\cos(0.8\lambda t),\,0.32\lambda\cos(0.4\lambda t),\,0]^\top$ m/s",
    "Circular":   r"$\,^\mathcal{I}\boldsymbol{v}_\text{t} = [0.125\lambda\sin(0.25\lambda t),\,0.125\lambda\cos(0.25\lambda t),\,0.1\cos(0.5\,t)]^\top$ m/s",
}


def draw_landing_corridor(ax, xt, yt, zt, half_xy=PRECISE_XY_M,
                          z_height=Z_F_M, color="0.6", alpha=0.10,
                          edge_color="k", edge_lw=0.5, edge_ls="--",
                          label=None):
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


def is_soft_precise(d, n):
    X = d.X_DS[:, :n]
    tgt = d.x_t[:3, :n]
    dtgt = d.dx_t[:3, :n] if hasattr(d, "dx_t") else np.zeros_like(tgt)
    xy_err = float(np.linalg.norm(X[:2, -1] - tgt[:2, -1]))
    vz_rel = float(abs(X[9, -1] - dtgt[2, -1]))
    return (xy_err <= PRECISE_XY_M) and (vz_rel <= SOFT_VZ_REL_MPS)


def plot_one_trajectory(traj):
    path = f"{DATA_DIR}/{traj}_multi_speed_comparison.mat"
    if not os.path.exists(path):
        return False
    m = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
    runs = np.atleast_1d(m["results"])

    fig = plt.figure(figsize=(8.5, 7.0))
    axes = [fig.add_subplot(2, 2, k + 1, projection="3d") for k in range(4)]
    colors = [viridis(i / (len(MULTS) - 1)) for i in range(len(MULTS))]

    for ax, ctrl_id in zip(axes, CTRL_ORDER):
        ctrl_runs = [r for r in runs if int(r.ctrl_id) == ctrl_id]
        # Sort by mult ascending
        ctrl_runs.sort(key=lambda r: float(r.mult))

        for li, run in enumerate(ctrl_runs):
            d = run.data
            N = int(d.idx) if int(d.idx) > 0 else d.X_DS.shape[1]
            xt = d.x_t[:, :N]
            draw_landing_corridor(ax, xt[0], xt[1], xt[2],
                                  alpha=0.10, edge_lw=0.4,
                                  label=(r"target ($\lambda=1.0$)"
                                         if abs(float(run.mult) - 1.0) < 1e-6
                                         else None))
        for li, run in enumerate(ctrl_runs):
            d = run.data
            N = int(d.idx) if int(d.idx) > 0 else d.X_DS.shape[1]
            X = d.X_DS[:, :N]
            mu = float(run.mult)
            c = colors[li]
            ax.plot(X[0], X[1], -X[2], color=c, lw=1.4,
                    label=rf"$\lambda={mu:.1f}$")
            end_marker = "^" if is_soft_precise(d, N) else "x"
            ax.scatter(X[0, -1], X[1, -1], -X[2, -1], color=c,
                       marker=end_marker, s=28, zorder=6)
        # IC marker (initial UAV position [0,0,-5] m → display altitude 5 m)
        ax.scatter(0, 0, 5, color="k", marker="o", s=30, zorder=7)

        ax.set_xlabel(r"$\,^\mathcal{I}x$ [m]", labelpad=2)
        ax.set_ylabel(r"$\,^\mathcal{I}y$ [m]", labelpad=2)
        ax.set_zlabel("altitude [m]", labelpad=2)
        ax.locator_params(axis="x", nbins=4)
        ax.locator_params(axis="y", nbins=4)
        ax.locator_params(axis="z", nbins=4)
        ax.tick_params(pad=1, labelsize=7)
        ax.set_title(CTRL_TITLE[ctrl_id], fontsize=8, y=1.0)
        ax.view_init(elev=22, azim=-58)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=6,
               bbox_to_anchor=(0.5, 0.0), frameon=False)
    fig.suptitle(f"{CASE_OF[traj]}: {traj} Target Trajectory\n{VEL_EQN[traj]}",
                 fontsize=10, y=0.99)
    # Layout: tight-fit subplots; then explicitly push their top boundary
    # up to 0.95 so the gap between suptitle bottom and subplot top is small.
    # wspace is set explicitly because tight_layout's 2D bbox detection
    # under-estimates the gutter needed for 3D-axes z-labels.
    fig.tight_layout(rect=(0, 0.05, 1, 1.0))
    fig.subplots_adjust(top=0.95, wspace=0.25)
    out_path = f"{OUT_DIR}/comparison_multi_speed_{traj.lower()}.pdf"
    # Note: do NOT use bbox_inches="tight" — it ignores 3D z-axis labels
    # (rendered in 3D coords) and crops them out of the right column.
    fig.savefig(out_path, pad_inches=0.05)
    plt.close(fig)
    print("Wrote", out_path)
    return True


def main():
    written = 0
    skipped = []
    for traj in TRAJS:
        if plot_one_trajectory(traj):
            written += 1
        else:
            skipped.append(traj)
    print(f"\nWrote {written} comparison multi-speed PDFs to {OUT_DIR}")
    if skipped:
        print("Skipped (missing .mat):")
        for s in skipped:
            print(f"  - {s}_multi_speed_comparison.mat")
        print("Run MATLAB/Comparison/multi_speed_comparison.m first.")


if __name__ == "__main__":
    main()
