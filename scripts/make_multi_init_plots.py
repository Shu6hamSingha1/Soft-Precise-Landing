"""
Generate uniform-style 3D and image-plane figures for the multi-init sweep
of the proposed PLASMC, replacing the legacy MATLAB-rendered PNGs that used
to live under Figures/11032026/.

For every trajectory in {Static, Linear, Sinusoidal, Lissajous, Circular}
and every condition in {realistic, noiseless} this script writes:

  Figures/generated/multi_init/<traj>_3D[_noiseless].pdf
  Figures/generated/multi_init/<traj>_image_plane[_noiseless].pdf

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
    # squeeze_me collapses a (1,1) struct array to a scalar struct;
    # normalise so callers always iterate over a list.
    r = m["results"]
    if not hasattr(r, "__len__"):
        m["results"] = [r]
    return m


def _idx_of(d):
    """Final logged step (clamped to array length so plots are clean)."""
    n_max = d.X_DS.shape[1]
    raw = int(getattr(d, "idx", n_max - 1))
    if raw <= 0:
        raw = n_max - 1
    return min(raw + 1, n_max)


def _land_idx(d):
    """Landing index (exclusive), or 0 if the run did not land.

    MATLAB zeros x_t at exactly ``idx``, so the target should be drawn
    up to ``idx`` (not idx+1) to avoid snapping back to the origin.
    """
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
    max_land = 0           # latest actual landing index (idx > 0)
    for k, run in enumerate(results):
        d = run.data
        n = _land_idx(d) or _idx_of(d)
        X = d.X_DS[:, :n]
        # Use actual initial position from state vector (NED: alt = -z)
        ic = X[:3, 0]
        ax.plot(X[0], X[1], -X[2],
                color=RUN_COLORS[k],
                lw=1.3,
                label=f"Run {k+1}: $({ic[0]:.0f},{ic[1]:.0f},{-ic[2]:.0f})$")
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
        # Trim target to the latest actual landing time so periodic
        # trajectories don't wrap back toward the origin.
        tgt_n = max_land if max_land > 0 else longest_n
        xt = longest.x_t[:, :tgt_n]
        ax.plot(xt[0], xt[1], -xt[2],
                color="k", lw=1.0, ls="--", label="Target")

    ax.set_xlabel(r"$\,^\mathcal{I}x$ [m]")
    ax.set_ylabel(r"$\,^\mathcal{I}y$ [m]")
    ax.set_zlabel("altitude [m]")
    ax.set_title(f"{traj} target ({cond_label})")
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

    n_runs = len(results)
    n_cols = int(np.ceil(np.sqrt(n_runs)))
    n_rows = int(np.ceil(n_runs / n_cols))
    fig, axes = plt.subplots(n_rows, n_cols,
                             figsize=(5.8, 4.8),
                             squeeze=False)

    for k, run in enumerate(results):
        ax = axes[k // n_cols, k % n_cols]
        d = run.data
        n = _land_idx(d) or _idx_of(d)

        # P_DS columns 9..12 are the raw pixel coordinates of the four
        # feature corners (cf. plot_multi_image_plane.m).
        P = d.P_DS[:, 8:12, :n]   # (2, 4, n)

        corner_colors = ["C0", "C1", "C2", "C4"]  # blue, orange, green, purple
        for i in range(4):
            x = P[0, i, :]
            y = P[1, i, :]
            c = corner_colors[i]
            ax.plot(x, y, color=c, lw=1.0,
                    label=f"corner {i+1}" if k == 0 else None)
            ax.plot(x[0], y[0], "o", color="k", ms=4,
                    mfc="k", mec="k",
                    label="start" if (i == 0 and k == 0) else None)
            ax.plot(x[-1], y[-1], "x", color="dimgray", ms=5, mew=1.4,
                    label="end"   if (i == 0 and k == 0) else None)

        # Desired pixels
        if hasattr(d, "V_nP_d"):
            Pd = d.V_nP_d
            ax.scatter(Pd[0, :], Pd[1, :], s=48, marker="*",
                       color="crimson", edgecolors="k", linewidths=0.4,
                       label="desired" if k == 0 else None, zorder=5)

        # Subplot title with actual IC (NED: alt = -z)
        ic = d.X_DS[:3, 0]
        ax.set_title(f"Run {k+1}: $({ic[0]:.0f},{ic[1]:.0f},{-ic[2]:.0f})$",
                     fontsize=7)
        ax.set_xlabel(r"$\hat{x}$ [px]", fontsize=7)
        ax.set_ylabel(r"$\hat{y}$ [px]", fontsize=7)
        ax.set_xlim(-160, 160)
        ax.set_ylim(-120, 120)
        ax.set_aspect("equal", adjustable="box")
        ax.tick_params(labelsize=6)

    # Hide any unused subplot cells
    for k in range(n_runs, n_rows * n_cols):
        axes[k // n_cols, k % n_cols].axis("off")

    fig.suptitle(f"{traj} target ({cond_label}) — image plane",
                 fontsize=10)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=4,
               fontsize=7, frameon=False, bbox_to_anchor=(0.5, -0.01))
    fig.tight_layout(rect=(0.0, 0.03, 1.0, 0.96), h_pad=0.8, w_pad=0.5)
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
