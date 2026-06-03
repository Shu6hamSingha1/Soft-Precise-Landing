"""
Multi-speed landing plots for the DF-ASMC target-speed sweep.

Reads:
  MATLAB/Datasets/MultiInit/<traj>_multi_speed.mat

For each of the four moving trajectories (Linear, Sinusoidal, Circular,
Lissajous) the sweep is a single IC [0,0,-5] with speed multipliers
{0.6, 0.8, 1.0, 1.2, 1.4}.

Outputs:
  Figures/generated/plasmc_multi_speed_landing.pdf

One subplot per trajectory (2x2 grid of 3D axes). Each subplot overlays
the UAV descent trajectory at all five speed multipliers, colored by a
Wong colorblind-friendly palette (cool→warm = slow→fast), with a faint dashed target path at lambda=1.0 for
reference and a marker at each touchdown point. The displayed target
altitude is offset by +0.2 m so the target line floats above the ground
plane. Touchdown markers follow the 5-category scheme of
`feedback_landing_marker_convention.md`: '^' filled triangle
(soft-precise), 'D' open diamond (precise only), 'o' open circle
(soft only), 'v' open down-triangle (touched down, neither),
'x' cross (failed to reach target surface).
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
    "font.serif": ["cmr10", "Computer Modern Roman", "DejaVu Serif"],
    "mathtext.fontset": "cm",
    "axes.formatter.use_mathtext": True,
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
DATA_DIR = f"{ROOT}/MATLAB/Datasets/MultiInit"
OUT_DIR  = f"{ROOT}/Soft_Precise_Landing/Figures/generated"
os.makedirs(OUT_DIR, exist_ok=True)

TRAJS = ["Linear", "Sinusoidal", "Lissajous", "Circular"]
MULTS = [0.6, 0.8, 1.0, 1.2, 1.4]
# Distinct ordered colors for the 5 speed multipliers (cool→warm to match
# slow→fast). Wong colorblind-friendly palette; replaces viridis since
# adjacent viridis samples were visually indistinguishable.
MULT_COLORS = ["#0072B2",  # blue   (λ=0.6, slowest)
               "#56B4E9",  # cyan   (λ=0.8)
               "#009E73",  # green  (λ=1.0, nominal)
               "#E69F00",  # orange (λ=1.2)
               "#D55E00"]  # vermillion (λ=1.4, fastest)

PRECISE_XY_M     = 0.08    # precise-landing horizontal threshold
SOFT_V_REL_MPS   = 0.20    # soft-landing 3-D relative-speed threshold
Z_F_M            = 0.20    # above-target gap at termination (corridor vertical height)


def draw_landing_corridor(ax, xt, yt, zt, half_xy=PRECISE_XY_M,
                          z_height=Z_F_M, color="0.6", alpha=0.18,
                          edge_color="k", edge_lw=0.6, edge_ls="--",
                          label=None):
    """3D corridor around the target trajectory representing the soft-precise
    allowable landing region: xy ±half_xy perpendicular to tangent; vertical
    from -zt (true target altitude) up to -zt + z_height (lifted target
    altitude) so the corridor follows target heave. Inputs in NED."""
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


def _classify_outcome(run):
    """Return (marker, filled) for the 5-category scheme — see
    `feedback_landing_marker_convention.md`. Reads MATLAB result-struct
    flags (`success`, `precise`, `soft`) directly."""
    if not bool(getattr(run, "success", False)):
        return ('x', True)
    p = bool(getattr(run, "precise", False))
    s = bool(getattr(run, "soft",    False))
    if   p and s: return ('^', True)
    elif p:       return ('D', False)
    elif s:       return ('o', False)
    else:         return ('v', False)


def _scatter_outcome(ax, x, y, z, color, run, s=28, zorder=6):
    marker, filled = _classify_outcome(run)
    if filled:
        ax.scatter(x, y, z, color=color, marker=marker, s=s, zorder=zorder)
    else:
        ax.scatter(x, y, z, facecolors='none', edgecolors=color,
                   marker=marker, s=s, linewidths=1.0, zorder=zorder)


def load_run(traj, tag=""):
    suffix = f"_{tag}" if tag else ""
    path = f"{DATA_DIR}/{traj}_multi_speed{suffix}.mat"
    m = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
    return np.atleast_1d(m["results"]), np.atleast_1d(m["mults"]).astype(float)


def plot_grid(tag, out_name):
    # Larger figure to host the 2x2 grid at the same per-axes font sizes used
    # in the manuscript's Circular_combined.pdf (label/title/legend = 20-24 pt).
    fig = plt.figure(figsize=(10.5, 10.5))
    axes = [fig.add_subplot(2, 2, k + 1, projection="3d") for k in range(4)]

    colors = MULT_COLORS

    for ax, traj in zip(axes, TRAJS):
        results, mults = load_run(traj, tag)
        mult_order = np.argsort(mults)

        run_N = []
        for i in mult_order:
            d = results[i].data
            run_N.append(int(d.idx) if int(d.idx) > 0 else d.X_DS.shape[1])

        # Per-lambda 3D landing corridor (xy ±0.08 m around each target
        # trajectory, vertical from true target altitude up to +z_f). Each
        # corridor is drawn at low alpha so the 5 stacked regions remain
        # readable. Only the nominal lambda=1.0 corridor gets a legend entry.
        for li, i in enumerate(mult_order):
            d = results[i].data
            N_i = run_N[li]
            xt_i = d.x_t[:, :N_i]
            is_nom = abs(mults[i] - 1.0) < 1e-6
            draw_landing_corridor(ax, xt_i[0], xt_i[1], xt_i[2],
                                  alpha=0.10,
                                  edge_lw=0.5,
                                  label=r"target ($\lambda=1.0$)"
                                        if is_nom else None)

        for idx_local, i in enumerate(mult_order):
            d = results[i].data
            N = run_N[idx_local]
            X = d.X_DS[:, :N]
            c = colors[idx_local]
            ax.plot(X[0], X[1], -X[2], color=c, lw=1.4,
                    label=rf"$\lambda={mults[i]:.1f}$")
            _scatter_outcome(ax, X[0, -1], X[1, -1], -X[2, -1],
                             c, results[i], s=28, zorder=6)
        ax.scatter(0, 0, 5, color="k", marker="o", s=30, zorder=7)
        # Match Circular_combined.pdf 3-D subplot styling.
        ax.set_xlabel(r"$\,^\mathcal{I}x$ [m]", labelpad=12, fontsize=20)
        ax.set_ylabel(r"$\,^\mathcal{I}y$ [m]", labelpad=12, fontsize=20)
        ax.set_zlabel("altitude [m]", labelpad=2, fontsize=20)
        ax.locator_params(axis="x", nbins=4)
        ax.locator_params(axis="y", nbins=4)
        ax.locator_params(axis="z", nbins=4)
        _case = {"Linear": "Case 2", "Sinusoidal": "Case 3",
                 "Circular": "Case 5", "Lissajous": "Case 4"}
        # Single-line subplot title; per-case velocity equations live in the
        # tex caption.
        ax.set_title(f"{_case[traj]}: {traj}", fontsize=20, y=0.95)
        ax.view_init(elev=22, azim=-58)
        ax.tick_params(pad=1, labelsize=18)

    # Shared legend at the bottom — same fontsize as the IC legend in
    # Circular_combined.pdf for visual uniformity.
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=6,
               bbox_to_anchor=(0.5, 0.0), frameon=False, fontsize=14,
               handlelength=1.6, columnspacing=2.0, handletextpad=0.6)
    # Suptitle in the same format as Circular_combined.pdf (fontsize=24, y=0.99).
    fig.suptitle("Landing at Multiple Speeds of Mobile Target Trajectories",
                 fontsize=24, y=0.99)
    # Tight everywhere: suptitle near top (y=0.99); subplots span almost the
    # full middle (top=0.94, bottom=0.07); rows share a common boundary
    # (hspace=0.0) so the inter-row gap collapses.
    fig.subplots_adjust(left=0.02, right=0.98, top=0.94, bottom=0.07,
                        wspace=0.0, hspace=0.0)
    for k, ax in enumerate(axes):
        pos = ax.get_position()
        is_top = k < 2
        dy = 0.02 if is_top else 0.0
        ax.set_position([pos.x0 - 0.02, pos.y0 + dy,
                         pos.width, pos.height])
    path = f"{OUT_DIR}/{out_name}"
    # bbox_inches="tight" drops 3D z-axis labels in matplotlib; use plain savefig.
    fig.savefig(path, pad_inches=0.05)
    plt.close(fig)
    print("Wrote", path)


if __name__ == "__main__":
    plot_grid(tag="",          out_name="plasmc_multi_speed_landing.pdf")
