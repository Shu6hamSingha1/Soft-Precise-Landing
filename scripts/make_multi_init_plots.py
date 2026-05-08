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
soft-precise allowable landing region. The 3D landing markers follow
the 5-category outcome scheme of `feedback_landing_marker_convention.md`:
'^' filled triangle (soft-precise), 'D' open diamond (precise only),
'o' open circle (soft only), 'v' open down-triangle (touched down,
neither), 'x' cross (failed to reach target surface).
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
SOFT_V_REL_MPS   = 0.20    # soft-landing 3-D relative-speed threshold
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


def _classify_outcome(run):
    """Return (marker, filled) for the 5-category scheme:
       ('^', True)  — soft-precise (success & precise & soft)
       ('D', False) — precise only  (success & precise & !soft)
       ('o', False) — soft only     (success & !precise & soft)
       ('v', False) — neither, but touched down
       ('x', True)  — failed to reach target surface (!success)
    Reads MATLAB run_simulation.m result-struct flags directly."""
    if not bool(getattr(run, "success", False)):
        return ('x', True)
    p = bool(getattr(run, "precise", False))
    s = bool(getattr(run, "soft",    False))
    if   p and s: return ('^', True)
    elif p:       return ('D', False)
    elif s:       return ('o', False)
    else:         return ('v', False)


def _scatter_outcome(ax, x, y, z, color, run):
    """Plot the touchdown marker per the 5-category scheme."""
    marker, filled = _classify_outcome(run)
    if filled:
        ax.scatter(x, y, z, color=color, marker=marker, s=30)
    else:
        ax.scatter(x, y, z, facecolors='none', edgecolors=color,
                   marker=marker, s=30, linewidths=1.0)


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
                label=rf"IC$_{k+1}$: $({ic[0]:.0f},{ic[1]:.0f},{-ic[2]:.0f})$")
        ax.scatter(X[0, 0], X[1, 0], -X[2, 0],
                   color=RUN_COLORS[k], marker="o", s=20)
        _scatter_outcome(ax, X[0, -1], X[1, -1], -X[2, -1],
                         RUN_COLORS[k], run)
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
        ic_label = (rf"IC$_{k+1}$: $({ic[0]:.0f},{ic[1]:.0f},{-ic[2]:.0f})$,"
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


def plot_combined(traj):
    """Single PDF combining 3-D trajectory (top) and image-plane (bottom) with
    one shared legend below — manuscript-grade layout for the headline figure.
    """
    from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset
    m = _load(traj)
    if m is None:
        return False
    results = m["results"]

    # Side-by-side layout: 3-D on the left, image-plane on the right.
    # gridspec with a tight left margin shifts subplot 1 further left
    # (covering the empty space) and a slightly wider wspace prevents the
    # 3-D panel's z-label from overlapping the image-plane panel.
    fig = plt.figure(figsize=(10.5, 6.0))
    # 3-D subplot needs a wider bbox than the image-plane subplot because
    # matplotlib leaves ~25-30 % internal horizontal padding around the
    # rendered cube. Width ratio 1.5:1.0 keeps the visible cube comparable
    # in size to the image-plane (which fills its bbox under set_aspect=equal).
    gs = fig.add_gridspec(1, 2, width_ratios=[1.4, 1.0],
                          left=0.05, right=0.98, wspace=0.10)
    ax3 = fig.add_subplot(gs[0, 0], projection="3d")
    axI = fig.add_subplot(gs[0, 1])

    # =========================================================================
    # Top: 3-D
    # =========================================================================
    longest = None
    longest_n = -1
    max_land = 0
    for k, run in enumerate(results):
        d = run.data
        n = _land_idx(d) or _idx_of(d)
        X = d.X_DS[:, :n]
        ic = X[:3, 0]
        ax3.plot(X[0], X[1], -X[2], color=RUN_COLORS[k], lw=1.3,
                 label=rf"IC$_{k+1}$: $({ic[0]:.0f},{ic[1]:.0f},{-ic[2]:.0f})$")
        ax3.scatter(X[0, 0], X[1, 0], -X[2, 0],
                    color=RUN_COLORS[k], marker="o", s=20)
        _scatter_outcome(ax3, X[0, -1], X[1, -1], -X[2, -1],
                         RUN_COLORS[k], run)
        if n > longest_n:
            longest_n = n
            longest = d
        li = _land_idx(d)
        if li > max_land:
            max_land = li
    if longest is not None:
        tgt_n = max_land if max_land > 0 else longest_n
        xt = longest.x_t[:, :tgt_n]
        draw_landing_corridor(ax3, xt[0], xt[1], xt[2],
                              label="Target corridor")

    ax3.set_xlabel(r"$\,^\mathcal{I}x$ [m]", labelpad=12, fontsize=20)
    ax3.set_ylabel(r"$\,^\mathcal{I}y$ [m]", labelpad=12, fontsize=20)
    ax3.set_zlabel("altitude [m]", labelpad=2, fontsize=20)
    ax3.locator_params(axis="x", nbins=4)
    ax3.locator_params(axis="y", nbins=4)
    ax3.locator_params(axis="z", nbins=4)
    ax3.tick_params(pad=1, labelsize=18)
    ax3.set_title("3-D View", fontsize=20, y=0.97)
    ax3.view_init(elev=22, azim=-58)

    # =========================================================================
    # Bottom: image plane
    # =========================================================================
    ic_colors = ["C0", "C1", "C2", "C3", "C4"]
    corner_styles = ["-", "--", "-.", ":"]

    def _closed_quad(px, py):
        return list(px) + [px[0]], list(py) + [py[0]]

    desired_quad = None
    end_corners  = []
    end_idx      = []
    max_offsets  = []
    for run in results:
        d = run.data
        n = _land_idx(d) or _idx_of(d)
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

    ic_handles = []
    for k, run in enumerate(results):
        d = run.data
        n_valid = end_idx[k] + 1
        P = d.P_DS[:, 8:12, :n_valid]
        c = ic_colors[k]
        ic = run.data.X_DS[:3, 0]
        for i in range(4):
            axI.plot(P[0, i, :], P[1, i, :], color=c, lw=0.7, alpha=0.5,
                     ls=corner_styles[i])
        sx, sy = _closed_quad(P[0, :, 0], P[1, :, 0])
        axI.plot(sx, sy, color=c, lw=1.2, alpha=0.4, ls="-", zorder=3)
        ex, ey = _closed_quad(end_corners[k][0], end_corners[k][1])
        ic_label = (rf"IC$_{k+1}$: $({ic[0]:.0f},{ic[1]:.0f},{-ic[2]:.0f})$,"
                    rf" $\Delta_{{\max}}={max_offsets[k]:.1f}$~px")
        h, = axI.plot(ex, ey, color=c, lw=1.4, ls=(0, (5, 2)), zorder=4,
                      label=ic_label)
        ic_handles.append(h)

    style_handles = []
    if desired_quad is not None:
        dxq, dyq = _closed_quad(desired_quad[0], desired_quad[1])
        h_des, = axI.plot(dxq, dyq, color="k", lw=2.0, ls="-", zorder=1,
                          alpha=0.45, label="desired")
        style_handles.append(h_des)
    h_start, = axI.plot([], [], color="gray", lw=1.2, alpha=0.4, label="start")
    h_end,   = axI.plot([], [], color="gray", lw=1.4, ls=(0, (5, 2)), label="end")
    style_handles += [h_start, h_end]

    axI.set_xlabel(r"$\,^\mathcal{C}\hat{x}$ [px]", fontsize=20)
    axI.set_ylabel(r"$\,^\mathcal{C}\hat{y}$ [px]", fontsize=20, labelpad=-6)
    axI.set_xlim(-160, 160)
    axI.set_ylim(-120, 120)
    axI.set_aspect("equal", adjustable="box")
    axI.tick_params(labelsize=18)
    axI.set_title("Image-Plane View", fontsize=20, y=1.03)

    # Inset on the converged region (unchanged from the standalone plot)
    if desired_quad is not None:
        dq_x = desired_quad[0]; dq_y = desired_quad[1]
        cx, cy = float(np.mean(dq_x)), float(np.mean(dq_y))
        dq_half = max(np.max(dq_x) - np.min(dq_x),
                      np.max(dq_y) - np.min(dq_y)) / 2.0
        pad = max([o for o in max_offsets if np.isfinite(o)] + [5.0]) + 5.0
        half = dq_half + pad
        ax_xmin, ax_xmax = -160.0, 160.0
        x0_frac = (50.0 - ax_xmin) / (ax_xmax - ax_xmin)
        w_frac = 1.0 - x0_frac
        h_frac = 0.57
        y0_frac = 1.0 - h_frac
        axins = axI.inset_axes([x0_frac, y0_frac, w_frac, h_frac])
        axins.set_xlim(cx - half, cx + half)
        axins.set_ylim(cy - half, cy + half)
        axins.set_aspect("equal")
        axins.tick_params(labelsize=10, pad=1)
        axins.locator_params(axis="x", nbins=3)
        axins.locator_params(axis="y", nbins=3)
        dxq, dyq = _closed_quad(desired_quad[0], desired_quad[1])
        axins.plot(dxq, dyq, color="k", lw=1.5, ls="-", zorder=1, alpha=0.45)
        for k in range(len(results)):
            ex, ey = _closed_quad(end_corners[k][0], end_corners[k][1])
            axins.plot(ex, ey, color=ic_colors[k], lw=1.2, ls=(0, (5, 2)),
                       zorder=4)
        mark_inset(axI, axins, loc1=2, loc2=4, fc="none", ec="0.6", lw=0.6)

    # =========================================================================
    # Legends:
    #   - Style legend (desired / start / end) inside the image-plane
    #     panel's upper-left corner (matches the standalone image_plane.pdf).
    #   - Merged IC legend at the bottom-center of the figure, 2 columns x
    #     3 rows (5 IC entries, last cell empty).
    # =========================================================================
    style_legend = axI.legend(handles=style_handles, loc="lower right",
                              fontsize=14, ncol=1, framealpha=0.9)
    axI.add_artist(style_legend)

    # 2-row IC legend (5 entries -> 2 rows x 3 cols, last cell empty),
    # fontsize 14, anchored at the figure bottom.
    fig.legend(handles=ic_handles, loc="lower center", ncol=3,
               fontsize=14, framealpha=0.9, bbox_to_anchor=(0.5, 0.0),
               handlelength=1.6, columnspacing=1.0, handletextpad=0.6)

    fig.suptitle(f"Landing for Multiple Initial Conditions for {TRAJ_CASE[traj]}",
                 fontsize=24, y=0.99)

    # Margins are FRACTIONS of figure height — they shrink the subplot
    # proportionally as figsize shrinks. Use small fractions so the suptitle
    # (~0.5 in) and the 2-row IC legend (~0.7 in) get exactly what they need
    # and the subplot fills the rest. At figsize=(11, 6.5) this yields a
    # subplot region ~5.1 in tall.
    fig.subplots_adjust(bottom=0.17, top=0.93)
    pos3 = ax3.get_position()
    ax3.set_position([pos3.x0 - 0.1, pos3.y0,
                      pos3.width, pos3.height])
    out = f"{OUT_DIR}/{traj}_combined.pdf"
    fig.savefig(out, pad_inches=0.05)
    plt.close(fig)
    return True


def main():
    # Only the combined (3-D + image-plane side-by-side) PDFs are used in the
    # manuscript and supplement. The legacy plot_3d() and plot_image_plane()
    # helpers are retained for future use but not invoked.
    written = 0
    skipped = []
    for traj in TRAJS:
        if plot_combined(traj):
            written += 1
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
