"""
Multi-speed landing plots for the PLASMC target-speed sweep.

Reads:
  MATLAB/Multi_init_cond/Datasets/<traj>_multi_speed.mat           -- realistic
  MATLAB/Multi_init_cond/Datasets/<traj>_multi_speed_noiseless.mat -- clean

For each of the four moving trajectories (Linear, Sinusoidal, Circular,
Lissajous) the sweep is a single IC [0,0,-5] with speed multipliers
{0.6, 0.8, 1.0, 1.2, 1.4}.

Outputs:
  Figures/generated/plasmc_multi_speed_landing.pdf
  Figures/generated/plasmc_multi_speed_landing_noiseless.pdf

One subplot per trajectory (2x2 grid of 3D axes). Each subplot overlays
the UAV descent trajectory at all five speed multipliers, colored by a
viridis colormap, with a faint dashed target path at lambda=1.0 for
reference and a marker at each touchdown point.
"""
import os
import numpy as np
import scipy.io as sio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.cm import viridis
from mpl_toolkits.mplot3d import Axes3D  # noqa

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
DATA_DIR = f"{ROOT}/MATLAB/Multi_init_cond/Datasets"
OUT_DIR  = f"{ROOT}/Soft_Precise_Landing/Figures/generated"
os.makedirs(OUT_DIR, exist_ok=True)

TRAJS = ["Linear", "Sinusoidal", "Circular", "Lissajous"]
MULTS = [0.6, 0.8, 1.0, 1.2, 1.4]


def target_xyz(traj, t, speed_mult=1.0):
    """Analytical target position, matching MATLAB/Common/traj_Gen.m.

    Note: the ship-deck heave sinusoid 0.2*sin(0.5 t) on Linear/Circular is
    *not* scaled by speed_mult (it comes from Lin 2022 and represents
    sea-state motion, independent of surge speed). The horizontal surge,
    circular angular rate, and Lissajous frequencies are all scaled by
    speed_mult.
    """
    lm = speed_mult
    if traj == "Linear":
        return np.stack([lm * t, lm * t, 0.2 * np.sin(0.5 * t)])
    if traj == "Sinusoidal":
        return np.stack([0.5 * np.sin(0.8 * lm * t),
                          0.3 * lm * t,
                          np.zeros_like(t)])
    if traj == "Circular":
        return np.stack([-0.5 * (np.cos(0.3 * lm * t) - 1),
                          0.5 * np.sin(0.3 * lm * t),
                          0.2 * np.sin(0.5 * t)])
    if traj == "Lissajous":
        return np.stack([0.5 * np.sin(-0.8 * lm * t),
                         0.8 * np.sin(0.4 * lm * t),
                         np.zeros_like(t)])
    raise ValueError(traj)


def load_run(traj, tag=""):
    suffix = f"_{tag}" if tag else ""
    path = f"{DATA_DIR}/{traj}_multi_speed{suffix}.mat"
    m = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
    return np.atleast_1d(m["results"]), np.atleast_1d(m["mults"]).astype(float)


def plot_grid(tag, out_name):
    fig = plt.figure(figsize=(8.5, 7.0))
    axes = [fig.add_subplot(2, 2, k + 1, projection="3d") for k in range(4)]

    colors = [viridis(i / (len(MULTS) - 1)) for i in range(len(MULTS))]

    for ax, traj in zip(axes, TRAJS):
        results, mults = load_run(traj, tag)
        mult_order = np.argsort(mults)

        run_N = []
        for i in mult_order:
            d = results[i].data
            run_N.append(int(d.idx) if int(d.idx) > 0 else d.X_DS.shape[1])

        nom_idx_local = next(li for li, i in enumerate(mult_order)
                              if abs(mults[i] - 1.0) < 1e-6)
        N_nom = run_N[nom_idx_local]
        dt    = float(results[mult_order[nom_idx_local]].data.tRange[1]
                     - results[mult_order[nom_idx_local]].data.tRange[0])

        # Nominal target (lambda=1): solid from t=0 to its own touchdown
        nom_d = results[mult_order[nom_idx_local]].data
        xt_nom = nom_d.x_t[:, :N_nom]
        ax.plot(xt_nom[0], xt_nom[1], -xt_nom[2],
                color="k", lw=1.0, ls="-",
                label=r"target ($\lambda=1.0$)")

        # For every run with lambda > 1.0, draw its own target trajectory as a
        # dashed line so the viewer can see how much further the target moved
        # during the longer chase at higher target speeds. The dashed segment
        # is computed analytically (nominal target formula scaled by lambda),
        # since the saved x_t is zero-padded past each run's touchdown.
        maxmult_idx = int(np.argmax(mults))
        for li, i in enumerate(mult_order):
            if mults[i] <= 1.0 + 1e-6:
                continue
            N_i = run_N[li]
            t_i = np.arange(N_i) * dt
            p_i = target_xyz(traj, t_i, speed_mult=float(mults[i]))
            label = rf"target ($\lambda={mults[i]:.1f}$)" if i == maxmult_idx else None
            ax.plot(p_i[0], p_i[1], -p_i[2],
                    color="k", lw=0.7, ls="--", alpha=0.7,
                    label=label)

        for idx_local, i in enumerate(mult_order):
            d = results[i].data
            N = run_N[idx_local]
            X = d.X_DS[:, :N]
            c = colors[idx_local]
            ax.plot(X[0], X[1], -X[2], color=c, lw=1.4,
                    label=rf"$\lambda={mults[i]:.1f}$")
            ax.scatter(X[0, -1], X[1, -1], -X[2, -1], color=c,
                       marker="x", s=28, zorder=6)
        ax.scatter(0, 0, 5, color="k", marker="o", s=30, zorder=7)
        ax.set_xlabel(r"$\,^\mathcal{I}x$ [m]", labelpad=-2)
        ax.set_ylabel(r"$\,^\mathcal{I}y$ [m]", labelpad=-2)
        ax.set_zlabel("altitude [m]", labelpad=-2)
        ax.set_title(traj)
        ax.view_init(elev=22, azim=-58)
        ax.tick_params(pad=-2)

    # Shared legend at the bottom
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=6,
               bbox_to_anchor=(0.5, 0.0), frameon=False)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    path = f"{OUT_DIR}/{out_name}"
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print("Wrote", path)


if __name__ == "__main__":
    plot_grid(tag="",          out_name="plasmc_multi_speed_landing.pdf")
    plot_grid(tag="noiseless", out_name="plasmc_multi_speed_landing_noiseless.pdf")
