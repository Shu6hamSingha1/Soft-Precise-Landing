"""
Per-trajectory multi-speed comparison plots for the four baseline controllers.

Mirrors the VDF-ASMC plot in plasmc_multi_speed_landing.pdf, but per-baseline
and per-trajectory: one PDF per moving trajectory, with a 2x2 grid of
subplots (Lin 2022, Zhang 2026, Lin 2023, Cho 2022). Each subplot overlays
the controller's UAV descent at five speed multipliers lambda in {0.6, 0.8,
1.0, 1.2, 1.4}, with per-lambda soft-precise allowable landing corridor and
5-category touchdown markers (see `feedback_landing_marker_convention.md`).

Reads:
  MATLAB/Datasets/Comparison/<traj>_multi_speed_comparison.mat
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
DATA_DIR = f"{ROOT}/MATLAB/Datasets/Comparison"
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

PRECISE_XY_M    = 0.08
SOFT_V_REL_MPS  = 0.20
Z_F_M           = 0.20

CTRL_ORDER  = [2, 3, 4, 5]
# Two CTRL_TITLE dicts because IEEE citation numbers differ between
# the main paper and supplement (separate bibliographies).
# ctrl_id maps to baseline letter:
#   2 -> Baseline A (Lin 2022)     main [1]  / supp [9]
#   3 -> Baseline B (Zhang 2026)   main [2]  / supp [8]
#   4 -> Baseline C (Lin 2023)     main [10] / supp [10]
#   5 -> Baseline D (Cho 2022)     main [9]  / supp [11]
CTRL_TITLE_MAIN = {2: "Baseline A [1] (PBVS--PPC)",
                   3: "Baseline B [2] (PBVS--AEDO)",
                   4: "Baseline C [10] (IBVS--PPC)",
                   5: "Baseline D [9] (FF--IBVS)"}
CTRL_TITLE_SUPP = {2: "Baseline A [9] (PBVS--PPC)",
                   3: "Baseline B [8] (PBVS--AEDO)",
                   4: "Baseline C [10] (IBVS--PPC)",
                   5: "Baseline D [11] (FF--IBVS)"}
# Circular trajectory PDF goes to main paper; the other three go to supplement.
TRAJ_IS_MAIN = {"Circular": True, "Linear": False,
                "Sinusoidal": False, "Lissajous": False}

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


def _classify_outcome(run):
    """5-category touchdown-marker scheme — see
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


def plot_one_trajectory(traj):
    path = f"{DATA_DIR}/{traj}_multi_speed_comparison.mat"
    if not os.path.exists(path):
        return False
    m = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
    runs = np.atleast_1d(m["results"])

    # Match the layout of plasmc_multi_speed_landing.pdf: larger figure,
    # 20-24 pt fonts, single-line suptitle (velocity eqn lives in caption).
    fig = plt.figure(figsize=(10.5, 10.5))
    axes = [fig.add_subplot(2, 2, k + 1, projection="3d") for k in range(4)]
    colors = MULT_COLORS
    # Citation-number variant depends on whether the figure is destined
    # for the main paper or supplement.
    ctrl_title = CTRL_TITLE_MAIN if TRAJ_IS_MAIN.get(traj, False) else CTRL_TITLE_SUPP

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
            _scatter_outcome(ax, X[0, -1], X[1, -1], -X[2, -1],
                             c, run, s=28, zorder=6)
        # IC marker (initial UAV position [0,0,-5] m → display altitude 5 m)
        ax.scatter(0, 0, 5, color="k", marker="o", s=30, zorder=7)

        ax.set_xlabel(r"$\,^\mathcal{I}x$ [m]", labelpad=12, fontsize=20)
        ax.set_ylabel(r"$\,^\mathcal{I}y$ [m]", labelpad=12, fontsize=20)
        ax.set_zlabel("altitude [m]", labelpad=2, fontsize=20)
        ax.locator_params(axis="x", nbins=4)
        ax.locator_params(axis="y", nbins=4)
        ax.locator_params(axis="z", nbins=4)
        ax.tick_params(pad=1, labelsize=18)
        ax.set_title(ctrl_title[ctrl_id], fontsize=20, y=0.95)
        ax.view_init(elev=22, azim=-58)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=6,
               bbox_to_anchor=(0.5, 0.0), frameon=False, fontsize=14,
               handlelength=1.6, columnspacing=2.0, handletextpad=0.6)
    # Single-line suptitle; the per-case velocity equation lives in the
    # tex caption (matches the plasmc_multi_speed_landing.pdf convention).
    fig.suptitle(f"Baseline Controllers at Multiple Target Speeds for {CASE_OF[traj]}",
                 fontsize=24, y=0.99)
    fig.subplots_adjust(left=0.02, right=0.98, top=0.94, bottom=0.07,
                        wspace=0.0, hspace=0.0)
    # Symmetric leftward shift; top row nudged up a touch.
    for k, ax in enumerate(axes):
        pos = ax.get_position()
        is_top = k < 2
        dy = 0.02 if is_top else 0.0
        ax.set_position([pos.x0 - 0.02, pos.y0 + dy,
                         pos.width, pos.height])
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
