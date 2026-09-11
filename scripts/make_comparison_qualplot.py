"""
make_comparison_qualplot.py

Rebuilds Figures/generated/comparison_qual_combined.pdf and adds a new
Figures/generated/comparison_outcome_heatmap.pdf.

RENAMED 2026-09-11 from comparison_combined_circular.pdf: that filename
collided with make_comparison_plots.py's "Plot H" output (both scripts wrote
the same path), and ICRA.tex's Fig. 4 is the make_comparison_plots.py
version -- this script's 2x2 was not currently wired into any .tex file.
Renamed to remove the collision; re-point a .tex file at this filename if
this 2x2 version is adopted there.

WHY (this script exists): the old comparison_combined_circular.pdf (see
make_comparison_plots.py, "Plot H") was a 1x4 of [3D Circular] + bars of
{landing time, final xy error, touchdown speed}. Once >=3 of the 4 baselines
abort on EVERY case (never reach the target surface), those three bars are
measured at the moment of abort, not at a landing -- "final xy error" and
"touchdown speed" for an airborne abort are not landing-precision/softness
numbers at all, and "landing time" is actively misleading (an early abort
reads as "fast").

REPLACED WITH (this script), all defined for every run regardless of outcome:
  1. 3D Circular (Case 5) trajectories -- also fixes a latent bug: the old
     _classify_outcome() read run.success/.precise/.soft, fields that do not
     exist on the saved comparison struct (only inside a differently-shaped
     .data), so every touchdown marker silently rendered as the 'x' (failed)
     glyph regardless of the actual outcome. Classified from data here.
  2. Bar: altitude above target at termination (all 5 cases) -- meaningful
     whether the run lands or aborts.
  3. Bar: terminal kinetic energy 1/2 m ||v_rel||^2 where the controller
     reaches the surface (bar omitted -- a gap -- where it aborts). The
     energy analog of "impact force": no ground-contact/compliance model is
     simulated, so a literal contact force isn't defined; kinetic energy is
     the honest, model-free severity proxy.
  4. FoV-margin time series, Case 5, all 5 controllers overlaid: the
     mechanism plot -- fraction of half-frame remaining before the farthest
     tracked feature exits, vs time.

comparison_outcome_heatmap.pdf: 5 (controller) x 5 (case) categorical grid
-- soft-precise / hard-imprecise / aborted.

Classification (thresholds match the rest of the manuscript):
  reached the surface <=> altitude above target at termination <= Z_REACH_M
  soft-precise         <=> reached AND r_xy <= PRECISE_XY_M AND v_term <= SOFT_V_REL_MPS
  hard/imprecise        <=> reached AND NOT soft-precise
  aborted               <=> NOT reached

NOTE 2026-09-11: "aborted" is deliberately mechanism-neutral, not "visibility
loss". Checking the run log against the FoV-margin panel found that most
Lin2023 cells never print "BREAK: FoV violation" -- they exit via one of the
un-logged safety clamps (norm(I_a_cd)>1e2, or a NaN guard on I_a_cd/u_2/x_c),
one iteration after the last logged sample (the same break-before-write
pattern fixed in _fov_margin below). The corrected margin trace confirms it:
on Case 5, Lin2023's margin dips to ~0.05-0.08 but never reaches zero before
the run ends, i.e. a command/acceleration divergence, not a literal FoV exit.
"Reached the surface: yes/no" is measured directly and is trustworthy; the
specific FAILURE MECHANISM per aborted cell is not verified here -- treat the
manuscript's "leaves the FoV" prose for baselines as needing that same
per-cell audit (log an explicit fov_fail flag alongside the break) before
citing a mechanism more specific than "did not reach the surface".
"""
import os
import numpy as np
import scipy.io as sio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
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
})

from pathlib import Path
ROOT = str(Path(__file__).resolve().parent.parent)
COMP = f"{ROOT}/MATLAB/Datasets/Comparison"
OUT  = f"{ROOT}/Soft_Precise_Landing/Figures/generated"
os.makedirs(OUT, exist_ok=True)


def safe_savefig(fig, target_path, **kwargs):
    """Save to a unique temp file, then replace the target -- a target open
    in a PDF viewer (Acrobat, etc.) locks the path against a direct write."""
    tmp_path = f"{os.path.dirname(target_path)}/.tmp_{os.getpid()}_{os.path.basename(target_path)}"
    fig.savefig(tmp_path, **kwargs)
    try:
        os.replace(tmp_path, target_path)
        print(f"Wrote {target_path}")
    except PermissionError:
        print(f"WARNING: {target_path} is locked (likely open in a viewer) -- "
              f"wrote {tmp_path} instead. Close it and re-run.")

RES = (320.0, 240.0)          # image resolution [px] (Common/Constants.m)
PRECISE_XY_M   = 0.08
SOFT_V_REL_MPS = 0.20
Z_REACH_M      = 0.21         # altitude-above-target at which the run counts as "reached"

TRAJS  = ["Static", "Linear", "Sinusoidal", "Lissajous", "Circular"]
LABELS = ["Case 1", "Case 2", "Case 3", "Case 4", "Case 5"]
CTRLS  = ["PLASMC (Proposed)", "Lin 2022", "Zhang 2026", "Lin 2023", "Cho 2022"]

CTRL_COLORS = {
    "PLASMC (Proposed)": "C3",
    "Lin 2022":          "C0",
    "Zhang 2026":        "C2",
    "Lin 2023":          "C4",
    "Cho 2022":          "C1",
}
# 2026-09-11: no author names, citation numbers, or method acronyms anywhere
# -- just "Proposed" / "Baseline A-D", per explicit user instruction, applied
# uniformly across all comparison figures (legends AND the heatmap's row
# labels below).
CTRL_DISPLAY = {
    "PLASMC (Proposed)": "Proposed",
    "Lin 2022":          "Baseline A",
    "Zhang 2026":        "Baseline B",
    "Lin 2023":          "Baseline C",
    "Cho 2022":          "Baseline D",
}
ROW_LABELS = {
    "PLASMC (Proposed)": "Proposed",
    "Lin 2022":          "Baseline A",
    "Zhang 2026":        "Baseline B",
    "Lin 2023":          "Baseline C",
    "Cho 2022":          "Baseline D",
}


def _run_metrics(run):
    """Per-run scalar metrics + outcome category from the saved comparison struct."""
    d = run.data
    N = int(d.idx) if int(d.idx) > 0 else d.tRange.shape[0]
    t   = d.tRange[:N]
    X   = d.X_DS[:, :N]
    xt  = d.x_t[:3, :N]
    dxt = d.dx_t[:3, :N] if hasattr(d, "dx_t") else np.zeros_like(X[:3])
    h_above = -X[2] - (-xt[2])                      # altitude above target [m]
    k = N - 1                                        # termination sample
    t_f    = float(t[k])
    h_term = float(h_above[k])
    r_xy   = float(np.linalg.norm(X[:2, k] - xt[:2, k]))
    v_term = float(np.linalg.norm(X[7:10, k] - dxt[:, k]))
    reached = h_term <= Z_REACH_M
    if not reached:
        cat = "aborted"
    elif r_xy <= PRECISE_XY_M and v_term <= SOFT_V_REL_MPS:
        cat = "soft-precise"
    else:
        cat = "hard-imprecise"
    m = float(getattr(d, "m", 2.114))
    ke = 0.5 * m * v_term ** 2 if reached else np.nan
    return dict(t_f=t_f, h_term=h_term, r_xy=r_xy, v_term=v_term, ke=ke,
                reached=reached, cat=cat, N=N, t=t, X=X, xt=xt)


def _fov_margin(run, N):
    """Per-timestep FoV margin: min over corners/axes of the fraction of
    half-frame remaining before the feature would exit (1 = centered, 0 =
    at the edge). Requires P_DS (run_comparison.m patched 2026-09-11 to
    save it; re-run the comparison if this raises).

    BUG FIX 2026-09-11: on an FoV-break run, the break in
    visualControl_comparison.m fires BEFORE that step's P_DS(:,:,idx) is
    written, so the column at N-1 is the zero-initialized default, not a
    real corner -- it reads as a spurious margin=1 (dead center) right at
    the moment the run actually terminated. Back-search for the last
    column with any nonzero corner (same pattern as
    make_multi_init_plots.py's _last_valid_p) and trim to it."""
    d = run.data
    P = d.P_DS[:, :, :N]
    Np = P.shape[1] // 3
    cnp = P[:, 2 * Np:3 * Np, :]                     # physical camera corners [px]
    j = cnp.shape[-1] - 1
    while j >= 0 and not np.any(cnp[:, :, j] != 0):
        j -= 1
    cnp = cnp[:, :, :j + 1]
    mx = (RES[0] / 2 - np.abs(cnp[0])) / (RES[0] / 2)
    my = (RES[1] / 2 - np.abs(cnp[1])) / (RES[1] / 2)
    return np.minimum(mx, my).min(axis=0)            # (j+1,)


def _closed_quad(px, py):
    return list(px) + [px[0]], list(py) + [py[0]]


def draw_landing_corridor(ax, xt, yt, zt, half_xy=PRECISE_XY_M, z_height=0.20,
                          color="0.6", alpha=0.18, edge_color="k", edge_lw=0.6, edge_ls="--"):
    xt = np.asarray(xt); yt = np.asarray(yt); zt = np.asarray(zt)
    zB = -zt; zT = -zt + z_height
    n = len(xt)
    dx = np.gradient(xt); dy = np.gradient(yt)
    mag = np.hypot(dx, dy)
    safe = np.where(mag < 1e-9, 1.0, mag)
    nx = -dy / safe; ny = dx / safe
    xL = xt - half_xy * nx; yL = yt - half_xy * ny
    xR = xt + half_xy * nx; yR = yt + half_xy * ny
    LT = list(zip(xL, yL, zT)); RT = list(zip(xR, yR, zT))
    LB = list(zip(xL, yL, zB)); RB = list(zip(xR, yR, zB))
    def strip(P, Q): return [[P[i], Q[i], Q[i + 1], P[i + 1]] for i in range(n - 1)]
    faces = strip(LT, RT) + strip(LB, RB) + strip(LT, LB) + strip(RT, RB)
    ax.add_collection3d(Poly3DCollection(faces, facecolor=color, alpha=alpha, edgecolor="none"))
    for X, Y, Z in [(xL, yL, zT), (xR, yR, zT), (xL, yL, zB), (xR, yR, zB)]:
        ax.plot(X, Y, Z, color=edge_color, lw=edge_lw, ls=edge_ls)


# ============================================================================
# Pass 1: compute every (traj, ctrl) metric once -- feeds both figures.
# ============================================================================
METRICS = {}     # (traj, ctrl_name) -> dict
RUNS    = {}     # (traj, ctrl_name) -> run object (for the 3D + margin panels)
for traj in TRAJS:
    mat = sio.loadmat(f"{COMP}/{traj}_comparison.mat", squeeze_me=True, struct_as_record=False)
    names = [str(x) for x in np.atleast_1d(mat["ctrl_names"])]
    runs  = np.atleast_1d(mat["all_results"])
    for name, run in zip(names, runs):
        METRICS[(traj, name)] = _run_metrics(run)
        RUNS[(traj, name)] = run

CASE5 = "Circular"
REACHED_CTRLS = [n for n in CTRLS if any(METRICS[(tr, n)]["reached"] for tr in TRAJS)]

CAT_CODE   = {"soft-precise": 0, "hard-imprecise": 1, "aborted": 2}
CAT_SYMBOL = {"soft-precise": "S", "hard-imprecise": "H", "aborted": "A"}   # S / H / A --
    # a checkmark glyph is missing from the cmr10 serif font used elsewhere in
    # the figure set (renders as a missing-glyph box); short text is robust
    # everywhere, incl. grayscale print.
CAT_COLORS = ["#2e7d32", "#f9a825", "#c62828"]   # green / amber / red


def _draw_heatmap(ax, title_fontsize=14, cell_fontsize=13, tick_fontsize=11, legend_fontsize=10,
                   show_legend=True, legend_ncol=1, legend_anchor=(0.0, -0.13), show_title=True):
    """5 (controller) x 5 (case) outcome grid. Shared by the standalone
    comparison_outcome_heatmap.pdf and panel (a) of the merged figure."""
    grid = np.array([[CAT_CODE[METRICS[(tr, name)]["cat"]] for tr in TRAJS] for name in CTRLS])
    cmap = ListedColormap(CAT_COLORS)
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N)
    ax.imshow(grid, cmap=cmap, norm=norm, aspect="auto")
    for i, name in enumerate(CTRLS):
        for j, tr in enumerate(TRAJS):
            cat = METRICS[(tr, name)]["cat"]
            ax.text(j, i, CAT_SYMBOL[cat], ha="center", va="center", fontsize=cell_fontsize,
                    fontweight="bold", color="white" if cat != "hard-imprecise" else "black")
    ax.set_xticks(range(len(TRAJS))); ax.set_xticklabels(LABELS, fontsize=tick_fontsize)
    ax.set_yticks(range(len(CTRLS))); ax.set_yticklabels([ROW_LABELS[n] for n in CTRLS], fontsize=tick_fontsize)
    ax.set_xticks(np.arange(-0.5, len(TRAJS), 1), minor=True)
    ax.set_yticks(np.arange(-0.5, len(CTRLS), 1), minor=True)
    ax.grid(which="minor", color="white", linewidth=2)
    ax.tick_params(which="minor", length=0)
    ax.tick_params(which="major", length=0)
    for spine in ax.spines.values():
        spine.set_visible(False)
    if show_title:
        ax.set_title("Closed-Loop Outcome by Controller and Case", fontsize=title_fontsize, pad=10)
    if show_legend:
        # legend_anchor="auto-left": align the legend's left edge with where
        # the row (controller-name) tick labels actually start, rather than
        # an eyeballed axes-fraction guess -- measure the leftmost tick
        # label's rendered bbox (requires a draw pass since text extents
        # aren't known until laid out) and convert it to this axes' fraction.
        if legend_anchor == "auto-left":
            fig = ax.figure
            fig.canvas.draw()
            renderer = fig.canvas.get_renderer()
            xs = [lbl.get_window_extent(renderer=renderer).x0 for lbl in ax.get_yticklabels()]
            x_anchor = 0.0
            if xs:
                inv = ax.transAxes.inverted()
                x_anchor, _ = inv.transform((min(xs), 0))
            legend_anchor = (x_anchor, legend_anchor[1] if isinstance(legend_anchor, tuple) else -0.07)
        legend_handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in CAT_COLORS]
        ax.legend(legend_handles, ["S: soft-precise", "H: hard/imprecise", "A: aborted"],
                  loc="upper left", bbox_to_anchor=legend_anchor, ncol=legend_ncol,
                  frameon=False, fontsize=legend_fontsize, columnspacing=1.2, handletextpad=0.5)


def _draw_3d(ax3d, fontsize=18, ticksize=13):
    """Circular (Case 5) 3D trajectories, outcome-classified touchdown markers."""
    target_drawn = False
    for name in CTRLS:
        run = RUNS[(CASE5, name)]
        met = METRICS[(CASE5, name)]
        d = run.data
        N = met["N"]
        X = d.X_DS[:, :N]
        color = CTRL_COLORS[name]
        ax3d.plot(X[0], X[1], -X[2], color=color, lw=1.3, label=CTRL_DISPLAY[name])
        ax3d.scatter(X[0, 0], X[1, 0], -X[2, 0], color=color, marker="o", s=20)
        marker = {"soft-precise": "^", "hard-imprecise": "o", "aborted": "x"}[met["cat"]]
        if met["cat"] == "hard-imprecise":
            ax3d.scatter(X[0, -1], X[1, -1], -X[2, -1], facecolors="none", edgecolors=color,
                         marker=marker, s=34, linewidths=1.2)
        else:
            ax3d.scatter(X[0, -1], X[1, -1], -X[2, -1], color=color, marker=marker, s=34)
        if not target_drawn:
            xt = d.x_t[:3, :N]
            draw_landing_corridor(ax3d, xt[0], xt[1], xt[2])
            target_drawn = True
    ax3d.set_xlabel(r"$\,^\mathcal{I}x$ [m]", labelpad=10, fontsize=fontsize)
    ax3d.set_ylabel(r"$\,^\mathcal{I}y$ [m]", labelpad=10, fontsize=fontsize)
    ax3d.set_zlabel("altitude [m]", labelpad=2, fontsize=fontsize)
    ax3d.locator_params(axis="x", nbins=4)
    ax3d.locator_params(axis="y", nbins=4)
    ax3d.locator_params(axis="z", nbins=4)
    ax3d.tick_params(pad=1, labelsize=ticksize)
    ax3d.view_init(elev=22, azim=-58)
    # No box_aspect/zoom override -- comparison_combined_circular.pdf's 3-D
    # panel (make_comparison_plots.py) doesn't use one either; it relies
    # purely on a near-square assigned box (4.32x4.20in) at the matplotlib
    # default zoom=1. THREED_AX_H is matched to AX3D_W for the same reason
    # here (see the vertical-stack comment above), so this panel now uses
    # the identical approach instead of compensating with zoom.


def _draw_energy(ax, fontsize=16, title_fontsize=17, tick_fontsize=14):
    """Relative touchdown energy eta_E = (v_f/v_soft)^2, plotted as
    log10(eta_E) so VISTA's near-zero bars and Zhang's 5-9x bars share one
    readable axis. Controllers that ever reach the surface only (VISTA,
    Zhang 2026) -- no bar slots wasted on controllers that never land."""
    xb = np.arange(len(TRAJS))
    width_b = 0.30
    xj_off = np.linspace(-0.5, 0.5, len(REACHED_CTRLS) + 2)[1:-1] * width_b * 2
    log_etas = []
    for j, name in enumerate(REACHED_CTRLS):
        eta = np.array([(METRICS[(tr, name)]["v_term"] / SOFT_V_REL_MPS) ** 2
                        if METRICS[(tr, name)]["reached"] else np.nan for tr in TRAJS])
        log_eta = np.log10(eta)
        log_etas.append(log_eta)
        mask = ~np.isnan(log_eta)
        xj = xb + xj_off[j]
        ax.bar(xj[mask], log_eta[mask], width_b, color=CTRL_COLORS[name], label=CTRL_DISPLAY[name])
    # N/A markers: horizontal "N/A" set in a small white box at the eta_E=1
    # (log=0) reference line -- previously rotated 90deg text sitting at the
    # bottom axis edge, which sat flush against the frame and was easy to
    # miss/clip. Centering on the y=0 gridline keeps every marker inside the
    # visible range regardless of the data's own log-scale extent.
    for j, name in enumerate(REACHED_CTRLS):
        eta = np.array([(METRICS[(tr, name)]["v_term"] / SOFT_V_REL_MPS) ** 2
                        if METRICS[(tr, name)]["reached"] else np.nan for tr in TRAJS])
        xj = xb + xj_off[j]
        for xk in xj[np.isnan(eta)]:
            ax.text(xk, 0.0, "N/A", rotation=90, ha="center", va="center",
                    fontsize=12, color=CTRL_COLORS[name], clip_on=False,
                    bbox=dict(boxstyle="round,pad=0.15", facecolor="white",
                              edgecolor=CTRL_COLORS[name], linewidth=0.6))
    # Straight (unrotated) case labels -- the in-panel corner note that used
    # to occupy this space is now a footnote below the panel instead (it
    # overlapped the "Only surface-reaching..." text against the topmost
    # bars), freeing this axis to sit flush.
    ax.set_xticks(xb); ax.set_xticklabels(LABELS, rotation=0, fontsize=tick_fontsize)
    ax.tick_params(axis="y", labelsize=tick_fontsize)
    ax.grid(axis="y", alpha=0.3)
    ax.set_ylabel(r"$\log_{10}(\eta_E)$", fontsize=fontsize, labelpad=4)
    # Sits in the x-label slot but at the shared bottom legend's font size
    # (well below the other panels' axis-label size) so it reads as a note,
    # not as this panel's actual x-axis label.
    ax.set_xlabel("Only surface-reaching runs shown", fontsize=18, style="italic", labelpad=6)


def _draw_fov_margin(ax, fontsize=16, tick_fontsize=14):
    """FoV-margin time series, Case 5, all 5 controllers -- the
    mechanism-explaining panel: why each baseline fails."""
    for name in CTRLS:
        run = RUNS[(CASE5, name)]
        N = METRICS[(CASE5, name)]["N"]
        margin = _fov_margin(run, N)            # may trim further than N -- see docstring
        t = run.data.tRange[:len(margin)]
        ax.plot(t, margin, color=CTRL_COLORS[name], lw=1.4, label=CTRL_DISPLAY[name])
    ax.set_ylim(bottom=min(-0.05, ax.get_ylim()[0]))
    ax.set_xlabel("$t$ [s]", fontsize=fontsize, labelpad=4)
    ax.set_ylabel("normalized FoV margin", fontsize=fontsize, labelpad=4)
    ax.tick_params(labelsize=tick_fontsize)
    ax.grid(alpha=0.3)


# ============================================================================
# Figure 1: comparison_qual_combined.pdf (2x2) -- the main comparison
#   figure, per the recommended set (2026-09-11): (a) outcome heatmap --
#   overall comparison, which controller lands/aborts/is hard/soft; (b) the
#   Circular (Case 5) 3D trajectory -- a representative qualitative case
#   study, the richest/most demanding case; (c) FoV margin, Case 5 -- WHY
#   baselines fail, not just that they do; (d) relative touchdown energy --
#   soft-landing quality as a severity measure, not a binary pass/fail.
# ============================================================================
#   Layout 2026-09-11, rewritten for simplicity: every panel is placed
#   directly by an explicit vertical "stack" of inch-height constants
#   (title space -> axes -> gap -> title space -> axes -> ... -> legend),
#   with the figure's total height DERIVED as the sum of that stack --
#   instead of the previous chain of gridspec-then-patch adjustments (three
#   different title conventions to hand-align, a fixed magic fig.text y that
#   had to be "re-verified" whenever anything above it moved, and leftover
#   canvas below the last row with no mechanism to reclaim it). Column
#   positions (X only) are unaffected by any of this and are reused verbatim
#   from the old gridspec's computed values.
#
#   Fontsize note (2026-09-11): included at \columnwidth same as multi_init's
#   Circular_combined.pdf (figsize width 10.5in, title/label fontsize 20) --
#   this canvas is 12.5in wide, so the same fontsize numbers print
#   10.5/12.5 = 0.84x smaller here. Every fontsize below is multiplied by
#   12.5/10.5 * (20/16) = 1.488 (the old titles were 16pt, not 20pt, so the
#   multiplier corrects both the width mismatch and the original undersize)
#   to land at multi_init's effective on-page size.
PANEL_TITLE_FS = 24   # shared across all four panel subtitles (16 x 1.488)

# Column X-positions (left, width), as fractions of figure width -- taken
# from a plain 2-column gridspec (width_ratios=[1.0, 1.25], wspace=0.28,
# left=0.13, right=0.98); unaffected by anything in the vertical stack below.
COL0_X0, COL0_W = 0.13000, 0.33138
COL1_X0, COL1_W = 0.56577, 0.41423
# 3-D axes pad heavily inside their own bbox; widen/left-shift column 1's
# box so the rendered cube actually fills it.
AX3D_X0, AX3D_W = COL1_X0 - 0.03, COL1_W + 0.05

# Vertical stack, top to bottom, in inches from the figure's top edge.
# Row 1 has UNEQUAL heights by design (user request): (a) shrunk, (b) grown
# as large as its column width comfortably supports -- (b) is NOT resized to
# close the gap under (a). One removable white space was TOP_MARGIN (blank
# canvas above the row-1 titles): comparison_combined_circular.pdf's
# reference layout (make_comparison_plots.py) puts its suptitle almost flush
# with the figure's top edge (y=0.99 of a 5in-tall figure, ~0.05in of
# margin) instead of a separate fixed margin -- shrunk to match that here.
#
# A second, larger one turned out to be inside (b) itself: matplotlib's
# Axes3D refuses to render taller than it is wide, so assigning
# THREED_AX_H > AX3D_W's inch width doesn't make the rendered cube any
# bigger -- verified numerically that 5.80in and 6.60in assigned heights
# produce the IDENTICAL 5.80x5.80in rendered content; the extra height was
# pure dead space between the title and the plot. THREED_AX_H is set to
# match AX3D_W below for exactly this reason: (b) looks identical, the gap
# inside it is gone, and the figure is shorter overall.
TOP_MARGIN    = 0.43     # bumped from 0.05: makes room for AX3D_SHIFT_UP below (see note there)
TITLE_H       = 0.45     # space reserved above each row for its subtitle
HM_AX_H       = 4.42     # panel (a) axes height -- grown to close the gap between (a)+legend
                          # and row 2 (capped below THREED_AX_H=5.80in per user instruction:
                          # (a) must not end up taller than (b))
HM_LEGEND_H   = 0.55     # room for (a)'s S/H/A legend, which hangs below its axes box
THREED_AX_H   = AX3D_W * 12.5   # panel (b) axes height, matched to its own width (see note above)
# (b)'s title (set_title(y=0.95), see below) sits INSET within its own box,
# well below the box's top edge -- unlike (a), whose title is placed by
# _panel_title in a reserved strip ABOVE its box. To land both titles at the
# same absolute height, (b)'s whole box is shifted up by the empirically
# measured gap between the two (0.831in, from comparing rendered title
# y-positions) -- TOP_MARGIN above was increased so this shift doesn't push
# the box off the top of the figure.
AX3D_SHIFT_UP = 0.831
ROW_GAP       = 0.60     # gap between row 1's lowest content and row 2's title
ROW2_AX_H     = 3.58     # panels (c)/(d) PLOT-AREA height (excludes x-tick/xlabel text below it); reduced from 4.47
ROW2_XLABEL_H = 0.75     # room for (c)/(d)'s x-tick labels + xlabel, which sit below ROW2_AX_H
LEGEND_GAP    = 0.15
LEGEND_H      = 0.45     # provisional -- corrected below from the actual render
BOTTOM_MARGIN = 0.10

row1_top    = TOP_MARGIN + TITLE_H
ax3d_top    = row1_top - AX3D_SHIFT_UP
row1_bottom = max(row1_top + HM_AX_H + HM_LEGEND_H, ax3d_top + THREED_AX_H)
row2_top    = row1_bottom + ROW_GAP + TITLE_H
row2_bottom = row2_top + ROW2_AX_H
legend_top  = row2_bottom + ROW2_XLABEL_H + LEGEND_GAP
FIG_H       = legend_top + LEGEND_H + BOTTOM_MARGIN


def _y0_frac(top_in, height_in):
    """(distance-from-top, height), both inches -> matplotlib's bottom-up y0 fraction."""
    return (FIG_H - top_in - height_in) / FIG_H


def _panel_title(ax, text):
    """Panel subtitle centered above ax, a fixed 0.10in above its top edge --
    used for ALL FOUR panels so they share one positioning rule (replaces
    three different prior conventions: a hand-tuned absolute fig.text y for
    (a), an axes-fraction y for the 3-D panel (b), and set_title(y=1.02-1.03)
    for (c)/(d)). Equal axes top edges (guaranteed by the stack above, since
    (a) and (b) both start at row1_top) now trivially give aligned titles --
    PROVIDED the axes' get_position(original=True) is used: Axes3D silently
    shrinks its box to preserve aspect and reports that shrunk "active" box
    from plain get_position(), which drifts further from the assigned box
    the more its assigned height deviates from its width (this is what threw
    (a)/(b) out of alignment once (b)'s height was made very different from
    (a)'s). original=True returns the box we actually assigned, for both 2-D
    and 3-D axes alike."""
    p = ax.get_position(original=True)
    ax.figure.text((p.x0 + p.x1) / 2, p.y1 + 0.10 / FIG_H, text,
                    ha="center", va="bottom", fontsize=PANEL_TITLE_FS)


fig = plt.figure(figsize=(12.5, FIG_H))

ax_hm = fig.add_axes([COL0_X0, _y0_frac(row1_top, HM_AX_H), COL0_W, HM_AX_H / FIG_H])
ax3d  = fig.add_axes([AX3D_X0, _y0_frac(ax3d_top, THREED_AX_H), AX3D_W, THREED_AX_H / FIG_H],
                      projection="3d")
ax_m  = fig.add_axes([COL0_X0, _y0_frac(row2_top, ROW2_AX_H), COL0_W, ROW2_AX_H / FIG_H])
ax_ke = fig.add_axes([COL1_X0, _y0_frac(row2_top, ROW2_AX_H), COL1_W, ROW2_AX_H / FIG_H])

# Category legend dropped here (redundant with the caption's S/H/A key and
# collided with the shared bottom controller-color legend); kept only on the
# standalone comparison_outcome_heatmap.pdf.
_draw_heatmap(ax_hm, cell_fontsize=19, tick_fontsize=16,
              show_legend=True, legend_ncol=3, legend_anchor="auto-left", legend_fontsize=15,
              show_title=False)
_draw_3d(ax3d, fontsize=25, ticksize=19)
_draw_fov_margin(ax_m, fontsize=24, tick_fontsize=21)
_draw_energy(ax_ke, fontsize=24, tick_fontsize=21)

# Titles placed after all four panels are drawn (order doesn't actually
# matter for alignment -- see _panel_title's original=True note above --
# but keeping them together here is clearer than interleaving draw/title
# calls per panel).
_panel_title(ax_hm, "(a) Closed-Loop Outcome")
# (b) is the one exception to the shared _panel_title helper: matching
# comparison_combined_circular.pdf's 3-D panel exactly, its title is placed
# WITH plain axes-relative set_title(y=0.95) instead of _panel_title's
# fixed pad above the box. A 3-D perspective view always leaves a naturally
# empty region near the top of its box (regardless of box aspect -- this is
# separate from the box-aspect fix above); the reference figure's tight
# look isn't a gap-free render, it's this empty region being used to hold
# the title text itself rather than reserving extra blank space above the
# box for it. Trade-off: (b)'s title baseline no longer matches (a)'s
# exactly (it now sits inset within (b)'s own box), same as the reference.
ax3d.set_title("(b) Landing Trajectories, Case 5", fontsize=PANEL_TITLE_FS,
                x=0.55, y=0.95)
_panel_title(ax_m, "(c) FoV Margin, Case 5")
_panel_title(ax_ke, "(d) Relative Touchdown Energy")

# Legend anchored just below row 2 (upper-center at legend_top), not at the
# absolute figure bottom -- ties its position to the content above it
# directly, instead of relying on a separately-sized blank canvas below.
# Single row (ncol=5): with the short "Proposed"/"Baseline A-D" labels this
# fits the canvas width comfortably.
handles, labels = ax3d.get_legend_handles_labels()
fig.legend(handles, labels, loc="upper center", ncol=5,
           bbox_to_anchor=(0.5, _y0_frac(legend_top, 0.0)),
           frameon=False, fontsize=18, handlelength=1.6, columnspacing=1.6, handletextpad=0.5)

safe_savefig(fig, f"{OUT}/comparison_qual_combined.pdf", pad_inches=0.05)
plt.close(fig)

# ============================================================================
# Figure 2: comparison_outcome_heatmap.pdf -- kept as a standalone (panel (a)
# of the merged figure above, at full size, for use outside the manuscript).
# ============================================================================
fig, ax = plt.subplots(figsize=(7.2, 3.6))
_draw_heatmap(ax)
fig.tight_layout()
safe_savefig(fig, f"{OUT}/comparison_outcome_heatmap.pdf", bbox_inches="tight", pad_inches=0.05)
plt.close(fig)

# ============================================================================
# Console summary (for the manuscript prose)
# ============================================================================
print("\n%-16s" % "controller", *["%-10s" % l for l in LABELS])
for name in CTRLS:
    row = [METRICS[(tr, name)]["cat"] for tr in TRAJS]
    print("%-16s" % ROW_LABELS[name], *["%-10s" % c for c in row])
