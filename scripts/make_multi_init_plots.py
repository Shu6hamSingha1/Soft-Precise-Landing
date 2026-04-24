"""
Generate uniform-style 3D and image-plane figures for the multi-init sweep
of the proposed DF-ASMC.

For every trajectory in {Static, Linear, Sinusoidal, Lissajous, Circular}
and every condition in {realistic, noiseless} this script writes:

  Figures/generated/multi_init/<traj>_3D[_noiseless].pdf
  Figures/generated/multi_init/<traj>_image_plane[_noiseless].pdf

Image-plane plots use a single axis with all 4 corners x 5 ICs overlaid,
plus the desired pixel location.

The realistic dataset is loaded from
  MATLAB/Multi_init_cond/Datasets/<traj>_multi_init.mat
and the clean dataset from
  MATLAB/Multi_init_cond/Datasets/<traj>_multi_init_noiseless.mat
both produced by Multi_init_cond/multi_Init_Var.m.
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
    "axes.grid": True,
    "grid.alpha": 0.3,
    "lines.linewidth": 1.2,
})

from pathlib import Path
ROOT      = str(Path(__file__).resolve().parent.parent)
DATA_DIR  = f"{ROOT}/MATLAB/Multi_init_cond/Datasets"
OUT_DIR   = f"{ROOT}/Soft_Precise_Landing/Figures/generated/multi_init"
os.makedirs(OUT_DIR, exist_ok=True)

TRAJS = ["Static", "Linear", "Sinusoidal", "Lissajous", "Circular"]
TRAJ_CASE = {"Static": "Case 1", "Linear": "Case 2", "Sinusoidal": "Case 3",
             "Lissajous": "Case 4", "Circular": "Case 5"}
TRAJ_TITLE = {"Static": "Static Target", "Linear": "Linear Target Trajectory",
              "Sinusoidal": "Sinusoidal Target Trajectory",
              "Lissajous": "Lissajous Target Trajectory",
              "Circular": "Circular Target Trajectory"}
CONDS = [
    ("realistic", ""),                  # <traj>_multi_init.mat
    ("noiseless", "_noiseless"),        # <traj>_multi_init_noiseless.mat
]
RUN_COLORS = ["C0", "C1", "C2", "C3", "C4"]


def _load(traj, suffix):
    path = f"{DATA_DIR}/{traj}_multi_init{suffix}.mat"
    if not os.path.exists(path):
        return None
    m = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
    r = m["results"]
    if not hasattr(r, "__len__"):
        m["results"] = [r]
    return m


def _idx_of(d):
    n_max = d.X_DS.shape[1]
    raw = int(getattr(d, "idx", n_max - 1))
    if raw <= 0:
        raw = n_max - 1
    return min(raw + 1, n_max)


def _land_idx(d):
    raw = int(getattr(d, "idx", 0))
    if raw <= 0:
        return 0
    n_max = d.X_DS.shape[1]
    return min(raw, n_max)


def plot_3d(traj, cond_label, suffix):
    m = _load(traj, suffix)
    if m is None:
        return False
    results = m["results"]
    fig = plt.figure(figsize=(4.5, 3.6))
    ax = fig.add_subplot(111, projection="3d")

    longest = None
    longest_n = -1
    max_land = 0
    for k, run in enumerate(results):
        d = run.data
        n = _land_idx(d) or _idx_of(d)
        X = d.X_DS[:, :n]
        ic = X[:3, 0]
        ax.plot(X[0], X[1], -X[2],
                color=RUN_COLORS[k],
                lw=1.3,
                label=f"IC{k+1}: $({ic[0]:.0f},{ic[1]:.0f},{-ic[2]:.0f})$")
        ax.scatter(X[0, 0], X[1, 0], -X[2, 0],
                   color=RUN_COLORS[k], marker="o", s=20)
        ax.scatter(X[0, -1], X[1, -1], -X[2, -1],
                   color=RUN_COLORS[k], marker="x", s=30)
        if n > longest_n:
            longest_n = n
            longest = d
        li = _land_idx(d)
        if li > max_land:
            max_land = li

    if longest is not None:
        tgt_n = max_land if max_land > 0 else longest_n
        xt = longest.x_t[:, :tgt_n]
        ax.plot(xt[0], xt[1], -xt[2],
                color="k", lw=1.0, ls="--", label="Target")

    ax.set_xlabel(r"$\,^\mathcal{I}x$ [m]")
    ax.set_ylabel(r"$\,^\mathcal{I}y$ [m]")
    ax.set_zlabel("altitude [m]")
    ax.set_title(f"{TRAJ_CASE[traj]}: {TRAJ_TITLE[traj]} ({cond_label})")
    ax.legend(loc="upper left", bbox_to_anchor=(0.0, 0.96),
              fontsize=6, ncol=1, framealpha=0.85)
    ax.view_init(elev=22, azim=-58)
    fig.tight_layout(pad=0.3)
    out = f"{OUT_DIR}/{traj}_3D{suffix}.pdf"
    fig.savefig(out, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    return True


def plot_image_plane(traj, cond_label, suffix):
    m = _load(traj, suffix)
    if m is None:
        return False
    results = m["results"]

    fig, ax = plt.subplots(figsize=(4.5, 3.6))

    ic_colors = ["C0", "C1", "C2", "C3", "C4"]
    corner_styles = ["-", "--", "-.", ":"]
    desired_drawn = False

    def _closed_quad(px, py):
        return list(px) + [px[0]], list(py) + [py[0]]

    for k, run in enumerate(results):
        d = run.data
        n = _land_idx(d) or _idx_of(d)
        P = d.P_DS[:, 8:12, :n]   # (2, 4, n)
        c = ic_colors[k]
        ic = d.X_DS[:3, 0]

        # Corner trajectories — color by IC, linestyle by corner
        for i in range(4):
            ax.plot(P[0, i, :], P[1, i, :], color=c, lw=0.7, alpha=0.5,
                    ls=corner_styles[i])

        # Start quad (solid, faded)
        sx, sy = _closed_quad(P[0, :, 0], P[1, :, 0])
        ax.plot(sx, sy, color=c, lw=1.2, alpha=0.4, ls="-", zorder=3)

        # End quad (long-dash, IC color)
        ex, ey = _closed_quad(P[0, :, -1], P[1, :, -1])
        ax.plot(ex, ey, color=c, lw=1.4, ls=(0, (5, 2)), zorder=4,
                label=f"IC{k+1}: $({ic[0]:.0f},{ic[1]:.0f},{-ic[2]:.0f})$")

        # Desired quad (drawn once)
        if not desired_drawn and hasattr(d, "V_nP_d"):
            Pd = d.V_nP_d
            dx, dy = _closed_quad(Pd[0, :], Pd[1, :])
            ax.plot(dx, dy, color="k", lw=2.0, ls="-", zorder=6,
                    label="desired")
            desired_drawn = True

    # Dummy entries for start/end style in legend
    ax.plot([], [], color="gray", lw=1.2, alpha=0.4, label="start")
    ax.plot([], [], color="gray", lw=1.4, ls=(0, (5, 2)), label="end")

    ax.set_xlabel(r"$\,^\mathcal{C}\hat{x}$ [px]")
    ax.set_ylabel(r"$\,^\mathcal{C}\hat{y}$ [px]")
    ax.set_xlim(-160, 160)
    ax.set_ylim(-120, 120)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(f"{TRAJ_CASE[traj]}: {TRAJ_TITLE[traj]} ({cond_label})")
    ax.legend(loc="lower right", fontsize=7, ncol=2, framealpha=0.85)
    fig.tight_layout()
    out = f"{OUT_DIR}/{traj}_image_plane{suffix}.pdf"
    fig.savefig(out, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    return True


def main():
    written = 0
    skipped = []
    for traj in TRAJS:
        for cond_label, suffix in CONDS:
            ok3 = plot_3d(traj, cond_label, suffix)
            okp = plot_image_plane(traj, cond_label, suffix)
            if ok3 and okp:
                written += 2
            else:
                skipped.append(f"{traj}{suffix}")

    print(f"Wrote {written} figures to {OUT_DIR}")
    if skipped:
        print("Skipped (missing .mat):")
        for s in skipped:
            print(f"  - {s}")
        print("Run MATLAB/Multi_init_cond/multi_Init_Var.m first.")


if __name__ == "__main__":
    main()
