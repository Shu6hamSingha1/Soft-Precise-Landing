"""
Generate uniform-style 3D and image-plane figures for the multi-init sweep
of the proposed DF-ASMC.

For every trajectory in {Static, Linear, Sinusoidal, Lissajous, Circular}
this script writes:

  Figures/generated/multi_init/<traj>_3D.pdf
  Figures/generated/multi_init/<traj>_image_plane.pdf

Image-plane plots use a single axis with all 4 corners x 5 ICs overlaid,
plus the desired pixel location.

Dataset loaded from
  MATLAB/Multi_init_cond/Datasets/<traj>_multi_init.mat
produced by Multi_init_cond/multi_Init_Var.m.

The 3D plots draw a target corridor (xy ±0.08 m around the trajectory,
vertical extent 0.20 m above the true target altitude) representing the
soft-precise allowable landing region. The 3D landing marker is a
triangle ('^') for soft-precise touchdowns (xy <= 0.08 m AND |v_z_rel| <=
0.20 m/s) and a cross ('x') otherwise.
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
RUN_COLORS = ["C0", "C1", "C2", "C3", "C4"]

PRECISE_XY_M     = 0.08    # precise-landing horizontal threshold
SOFT_VZ_REL_MPS  = 0.20    # soft-landing relative vertical-speed threshold
Z_F_M            = 0.20    # above-target gap at termination (corridor vertical height)


def _load(traj):
    path = f"{DATA_DIR}/{traj}_multi_init.mat"
    if not os.path.exists(path):
        return None
    m = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
    r = m["results"]
    if not hasattr(r, "__len__"):
        m["results"] = [r]
    return m


def draw_landing_corridor(ax, xt, yt, zt, half_xy=PRECISE_XY_M,
                          z_height=Z_F_M, color="0.6", alpha=0.18,
                          edge_color="k", edge_lw=0.6, edge_ls="--",
                          label=None):
    """Draw a 3D corridor representing the soft-precise allowable landing
    region around the target trajectory:
      - horizontal: ±half_xy m perpendicular to the trajectory tangent in xy
      - vertical:   from the true target altitude (-zt) up to (-zt + z_height),
                    so the corridor follows target heave
    Static targets (near-zero motion) are drawn as a vertical cuboid disc.
    Inputs (xt, yt, zt) are NED; the function converts zt to display altitude.
    """
    xt = np.asarray(xt); yt = np.asarray(yt); zt = np.asarray(zt)
    zB = -zt                  # bottom of corridor: true target display altitude
    zT = -zt + z_height       # top:    lifted target display altitude
    n = len(xt)
    if n < 2:
        return
    dx = np.gradient(xt); dy = np.gradient(yt)
    mag = np.hypot(dx, dy)
    if np.max(mag) < 1e-3:
        # Static target: vertical square prism at (xt[0], yt[0])
        sx = np.array([-1, 1, 1, -1]) * half_xy + xt[0]
        sy = np.array([-1, -1, 1, 1]) * half_xy + yt[0]
        zT0 = zT[0]; zB0 = zB[0]
        top    = list(zip(sx, sy, np.full(4, zT0)))
        bottom = list(zip(sx, sy, np.full(4, zB0)))
        sides = []
        for i in range(4):
            j = (i + 1) % 4
            sides.append([top[i], top[j], bottom[j], bottom[i]])
        ax.add_collection3d(Poly3DCollection(
            [top, bottom] + sides, facecolor=color, alpha=alpha,
            edgecolor="none"))
        # Top outline
        ax.plot([*sx, sx[0]], [*sy, sy[0]],
                np.full(5, zT0), color=edge_color, lw=edge_lw, ls=edge_ls,
                label=label)
        return
    safe = np.where(mag < 1e-9, 1.0, mag)
    nx = -dy / safe; ny = dx / safe                # xy perpendicular to tangent
    xL = xt - half_xy * nx; yL = yt - half_xy * ny
    xR = xt + half_xy * nx; yR = yt + half_xy * ny
    LT = list(zip(xL, yL, zT))   # left-top corner curve
    RT = list(zip(xR, yR, zT))   # right-top
    LB = list(zip(xL, yL, zB))   # left-bottom
    RB = list(zip(xR, yR, zB))   # right-bottom
    def strip(P, Q):
        return [[P[i], Q[i], Q[i+1], P[i+1]] for i in range(n - 1)]
    faces = (
        strip(LT, RT) +   # top face (lifted target ribbon)
        strip(LB, RB) +   # bottom face (true target ribbon)
        strip(LT, LB) +   # left wall
        strip(RT, RB)     # right wall
    )
    ax.add_collection3d(Poly3DCollection(faces, facecolor=color, alpha=alpha,
                                         edgecolor="none"))
    # 4 corner outlines
    ax.plot(xL, yL, zT, color=edge_color, lw=edge_lw, ls=edge_ls, label=label)
    ax.plot(xR, yR, zT, color=edge_color, lw=edge_lw, ls=edge_ls)
    ax.plot(xL, yL, zB, color=edge_color, lw=edge_lw, ls=edge_ls)
    ax.plot(xR, yR, zB, color=edge_color, lw=edge_lw, ls=edge_ls)


def _last_valid_p(P_DS, n_max):
    """Return the largest j < n_max such that P_DS[:, 8:12, j] has any
    non-zero entry. Some runs zero-pad P_DS earlier than the saved `idx`,
    making P_DS[:, 8:12, idx-1] vanish — back-search guarantees we
    snapshot the actual last touchdown sample."""
    j = min(n_max, P_DS.shape[2]) - 1
    while j >= 0 and not np.any(P_DS[:, 8:12, j] != 0):
        j -= 1
    return max(j, 0)


def _is_soft_precise(d, n):
    """Return True iff the touchdown sample (index n-1) satisfies the
    soft+precise criterion (xy <= 0.08 m AND |v_z_rel| <= 0.20 m/s)."""
    X   = d.X_DS[:, :n]
    tgt = d.x_t[:3, :n]
    dtgt = d.dx_t[:3, :n] if hasattr(d, "dx_t") else np.zeros_like(tgt)
    xy_err = float(np.linalg.norm(X[:2, -1] - tgt[:2, -1]))
    vz_rel = float(abs(X[9, -1] - dtgt[2, -1]))
    return (xy_err <= PRECISE_XY_M) and (vz_rel <= SOFT_VZ_REL_MPS)


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


def plot_3d(traj):
    m = _load(traj)
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
        end_marker = "^" if _is_soft_precise(d, n) else "x"
        ax.scatter(X[0, -1], X[1, -1], -X[2, -1],
                   color=RUN_COLORS[k], marker=end_marker, s=30)
        if n > longest_n:
            longest_n = n
            longest = d
        li = _land_idx(d)
        if li > max_land:
            max_land = li

    if longest is not None:
        tgt_n = max_land if max_land > 0 else longest_n
        xt = longest.x_t[:, :tgt_n]
        draw_landing_corridor(ax, xt[0], xt[1], xt[2],
                              label="Target corridor")

    ax.set_xlabel(r"$\,^\mathcal{I}x$ [m]", labelpad=2)
    ax.set_ylabel(r"$\,^\mathcal{I}y$ [m]", labelpad=2)
    ax.set_zlabel("altitude [m]", labelpad=2)
    ax.locator_params(axis="x", nbins=4)
    ax.locator_params(axis="y", nbins=4)
    ax.locator_params(axis="z", nbins=4)
    ax.tick_params(pad=1, labelsize=7)
    ax.set_title(f"{TRAJ_CASE[traj]}: {TRAJ_TITLE[traj]}", fontsize=8, y=0.95)
    # Vertical legend OUTSIDE the axes, anchored well past the z-label so it
    # does not overlap. Pair with subplots_adjust(right=0.66) to reserve room.
    ax.legend(loc="center left", bbox_to_anchor=(1.20, 0.5),
              fontsize=6, ncol=1, framealpha=0.85)
    ax.view_init(elev=22, azim=-58)
    fig.tight_layout(pad=0.3)
    fig.subplots_adjust(right=0.66)
    out = f"{OUT_DIR}/{traj}_3D.pdf"
    fig.savefig(out, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    return True


def plot_image_plane(traj):
    from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset

    m = _load(traj)
    if m is None:
        return False
    results = m["results"]

    fig, ax = plt.subplots(figsize=(4.5, 3.6))

    ic_colors = ["C0", "C1", "C2", "C3", "C4"]
    corner_styles = ["-", "--", "-.", ":"]

    def _closed_quad(px, py):
        return list(px) + [px[0]], list(py) + [py[0]]

    # Pass 1: gather data, find desired quad, compute per-IC max corner-to-
    # desired offset (used for legend annotation and inset zoom range).
    # NB: some runs zero-pad P_DS BEFORE the saved `idx`, so we back-search
    # for the last non-zero P_DS sample rather than blindly indexing n-1.
    desired_quad = None
    end_corners  = []   # list of (P_x_end, P_y_end) per IC
    end_idx      = []   # last-valid sample index per IC (for trimming the trail)
    max_offsets  = []
    ics          = []
    nlist        = []
    for k, run in enumerate(results):
        d = run.data
        n = _land_idx(d) or _idx_of(d)
        nlist.append(n)
        ics.append(d.X_DS[:3, 0])
        j_end = _last_valid_p(d.P_DS, n)
        end_idx.append(j_end)
        end_corners.append((d.P_DS[0, 8:12, j_end].copy(),
                            d.P_DS[1, 8:12, j_end].copy()))
        if desired_quad is None and hasattr(d, "V_nP_d"):
            Pd = d.V_nP_d
            desired_quad = (np.asarray(Pd[0, :]).copy(),
                            np.asarray(Pd[1, :]).copy())
    for k in range(len(results)):
        if desired_quad is not None:
            dx_c = end_corners[k][0] - desired_quad[0]
            dy_c = end_corners[k][1] - desired_quad[1]
            max_offsets.append(float(np.max(np.hypot(dx_c, dy_c))))
        else:
            max_offsets.append(np.nan)

    # MAIN AXES
    ic_handles = []
    for k, run in enumerate(results):
        d = run.data
        n_valid = end_idx[k] + 1     # trim past the zero-padded tail
        P = d.P_DS[:, 8:12, :n_valid]
        c = ic_colors[k]
        ic = ics[k]

        # Corner trajectories — color by IC, linestyle by corner
        for i in range(4):
            ax.plot(P[0, i, :], P[1, i, :], color=c, lw=0.7, alpha=0.5,
                    ls=corner_styles[i])

        # Start quad (solid, faded)
        sx, sy = _closed_quad(P[0, :, 0], P[1, :, 0])
        ax.plot(sx, sy, color=c, lw=1.2, alpha=0.4, ls="-", zorder=3)

        # End quad (long-dash, IC color) — uses the back-searched last sample
        ex, ey = _closed_quad(end_corners[k][0], end_corners[k][1])
        ic_label = (rf"IC{k+1}: $({ic[0]:.0f},{ic[1]:.0f},{-ic[2]:.0f})$,"
                    rf" $\Delta_{{\max}}={max_offsets[k]:.1f}$~px")
        h, = ax.plot(ex, ey, color=c, lw=1.4, ls=(0, (5, 2)), zorder=4,
                     label=ic_label)
        ic_handles.append(h)

    # Desired quad — drawn UNDER the IC end quads (low zorder, translucent)
    style_handles = []
    if desired_quad is not None:
        dxq, dyq = _closed_quad(desired_quad[0], desired_quad[1])
        h_des, = ax.plot(dxq, dyq, color="k", lw=2.0, ls="-", zorder=1,
                         alpha=0.45, label="desired")
        style_handles.append(h_des)

    # Dummy line artists for start/end style entries
    h_start, = ax.plot([], [], color="gray", lw=1.2, alpha=0.4, label="start")
    h_end,   = ax.plot([], [], color="gray", lw=1.4, ls=(0, (5, 2)), label="end")
    style_handles += [h_start, h_end]

    ax.set_xlabel(r"$\,^\mathcal{C}\hat{x}$ [px]")
    ax.set_ylabel(r"$\,^\mathcal{C}\hat{y}$ [px]")
    ax.set_xlim(-160, 160)
    ax.set_ylim(-120, 120)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(f"{TRAJ_CASE[traj]}: {TRAJ_TITLE[traj]}")

    # Two stacked legends:
    # - Legend 1 (style):   desired / start / end, upper-LEFT (empty space)
    # - Legend 2 (per-IC):  5 IC entries with max-offset, lower-RIGHT
    legend1 = ax.legend(handles=style_handles, loc="upper left",
                        fontsize=6, ncol=1, framealpha=0.9)
    ax.add_artist(legend1)
    ax.legend(handles=ic_handles, loc="lower right",
              fontsize=6, ncol=1, framealpha=0.9)

    # INSET — top-right empty space, zoomed on the converged region
    if desired_quad is not None:
        dq_x = desired_quad[0]; dq_y = desired_quad[1]
        cx, cy = float(np.mean(dq_x)), float(np.mean(dq_y))
        dq_half = max(np.max(dq_x) - np.min(dq_x),
                      np.max(dq_y) - np.min(dq_y)) / 2.0
        pad = max([o for o in max_offsets if np.isfinite(o)] + [5.0]) + 5.0
        half = dq_half + pad
        xl, xr = cx - half, cx + half
        yl, yr = cy - half, cy + half

        # Anchor inset's LEFT edge at image-plane x=50 px (data coord); right
        # edge at the right of the axes (x=160 px). Vertical: top of axes
        # down by the existing 57% height.
        ax_xmin, ax_xmax = -160.0, 160.0
        x0_frac = (50.0 - ax_xmin) / (ax_xmax - ax_xmin)
        w_frac  = 1.0 - x0_frac
        h_frac  = 0.57
        y0_frac = 1.0 - h_frac
        axins = ax.inset_axes([x0_frac, y0_frac, w_frac, h_frac])
        axins.set_xlim(xl, xr)
        axins.set_ylim(yl, yr)
        axins.set_aspect("equal")
        axins.tick_params(labelsize=5, pad=1)
        axins.locator_params(axis="x", nbins=3)
        axins.locator_params(axis="y", nbins=3)

        # Re-draw desired (under) and end quads (over) inside the inset
        dxq, dyq = _closed_quad(desired_quad[0], desired_quad[1])
        axins.plot(dxq, dyq, color="k", lw=1.5, ls="-", zorder=1, alpha=0.45)
        for k in range(len(results)):
            ex, ey = _closed_quad(end_corners[k][0], end_corners[k][1])
            axins.plot(ex, ey, color=ic_colors[k], lw=1.2, ls=(0, (5, 2)),
                       zorder=4)
        # Pixel-value annotations removed — Δ_max already in the IC legend.

        # Connector lines from inset (upper-left + lower-right corners) to
        # the highlighted rectangle on the main axes — visual zoom indicator.
        mark_inset(ax, axins, loc1=2, loc2=4, fc="none", ec="0.6", lw=0.6)

    fig.tight_layout()
    out = f"{OUT_DIR}/{traj}_image_plane.pdf"
    fig.savefig(out, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    return True


def main():
    written = 0
    skipped = []
    for traj in TRAJS:
        ok3 = plot_3d(traj)
        okp = plot_image_plane(traj)
        if ok3 and okp:
            written += 2
        else:
            skipped.append(traj)

    print(f"Wrote {written} figures to {OUT_DIR}")
    if skipped:
        print("Skipped (missing .mat):")
        for s in skipped:
            print(f"  - {s}")
        print("Run MATLAB/Multi_init_cond/multi_Init_Var.m first.")


if __name__ == "__main__":
    main()
