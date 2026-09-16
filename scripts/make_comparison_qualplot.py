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
    "figure.dpi": 600,
    "savefig.dpi": 600,
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
    "pdf.fonttype": 42,   # embed text/math as scalable Type 42 (TrueType), not Type 3 --
    "ps.fonttype": 42,    # ICRA/IEEE PDF checkers reject Type 3 fonts (mathtext's cm fontset
                          # defaults to Type 3 bitmaps otherwise)
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
F_PX = 135.0                  # camera focal length [px] (Common/Constants.m: f=135)
CBF_BUFFER_FRAC = 0.15        # b: FoV-edge buffer (vdf_params.m P.cbf_buffer_frac)
PHI_MAX = (np.array(RES) / 2.0 / F_PX) * (1.0 - CBF_BUFFER_FRAC)  # [phi_x,max, phi_y,max]
PRECISE_XY_M   = 0.08
SOFT_V_REL_MPS = 0.20
Z_REACH_M      = 0.21         # altitude-above-target at which the run counts as "reached"

TRAJS  = ["Static", "Linear", "Sinusoidal", "Lissajous", "Circular"]
LABELS = ["Case 1", "Case 2", "Case 3", "Case 4", "Case 5"]
CTRLS  = ["PLASMC (Proposed)", "Lin 2022", "Zhang 2026", "Lin 2023", "Cho 2022"]

CTRL_COLORS = {
    "PLASMC (Proposed)": "#D55E00",
    "Lin 2022":          "#0072B2",
    "Zhang 2026":        "#009E73",
    "Lin 2023":          "#CC79A7",
    "Cho 2022":          "#E69F00",
}
# 2026-09-11: no author names or citation numbers anywhere, per explicit user
# instruction, applied uniformly across all comparison figures (legends AND
# the heatmap's row labels below).
# 2026-09-15: "Baseline A-D" swapped for method-acronym tags per explicit
# user instruction (supersedes the 2026-09-11 "Baseline A-D only" call).
CTRL_DISPLAY = {
    "PLASMC (Proposed)": "Proposed",
    "Lin 2022":          "PBVS-PPC",
    "Zhang 2026":        "PBVS-AEDO",
    "Lin 2023":          "IBVS-PPC",
    "Cho 2022":          "FF-IBVS",
}
ROW_LABELS = {
    "PLASMC (Proposed)": "Proposed",
    "Lin 2022":          "PBVS-PPC",
    "Zhang 2026":        "PBVS-AEDO",
    "Lin 2023":          "IBVS-PPC",
    "Cho 2022":          "FF-IBVS",
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
    """Per-timestep barrier h_k({}^C r-tilde) = 1 - |[Phi^-1 {}^C r-tilde]_k|,
    k in {x,y} (ICRA.tex cbf barrier: equation), evaluated at the MARKER
    CENTRE (mean of the 4 arm tips, excluding the stub -- same convention as
    make_multi_init_plots.py's _marker_centre) and plotted as min_k h_k(t).
    Phi^-1 normalizes by the buffered, focal-length-normalized FoV half-width
    phi_max = (R/2/f)(1-b) (f=135px, b=0.15, both from the live controller
    constants -- see PHI_MAX above), so this is the exact theorem quantity,
    not a raw-pixel analog. 1 = centred, 0 = at the buffered FoV edge
    (negative = past the buffer, inside the true sensor edge only if
    |value| < b-dependent slack -- the buffer is the margin the CBF holds
    open, not the physical sensor boundary itself).

    CORRECTED 2026-09-16 (two fixes): (1) evaluated at the marker centre
    only, not min'd over all tracked corners (wrong for the cross+stub
    marker -- only the centre is theory-relevant, same reasoning as the
    Delta_c fix in make_multi_init_plots.py); (2) uses the actual buffered,
    Phi-normalized barrier instead of a raw-pixel (half-frame-relative,
    unbuffered) stand-in, so the plotted symbol min_k h_k(t) now matches
    ICRA.tex's h_k exactly rather than by structural analogy only.
    Requires P_DS (run_comparison.m patched 2026-09-11 to save it; re-run
    the comparison if this raises).

    BUG FIX 2026-09-11 (still applies): on an FoV-break run, the break in
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
    n_arms = min(4, Np)                              # exclude the stub (col 5)
    cx = cnp[0, :n_arms, :].mean(axis=0)              # marker-centre x [px]
    cy = cnp[1, :n_arms, :].mean(axis=0)              # marker-centre y [px]
    rx = cx / F_PX                                    # tangent-space r-tilde_x
    ry = cy / F_PX                                    # tangent-space r-tilde_y
    hx = 1.0 - np.abs(rx / PHI_MAX[0])
    hy = 1.0 - np.abs(ry / PHI_MAX[1])
    return rx, ry, np.minimum(hx, hy)                 # each (j+1,)


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
        ax3d.plot(X[0], X[1], -X[2], color=color, lw=2.2, label=CTRL_DISPLAY[name])
        ax3d.scatter(X[0, 0], X[1, 0], -X[2, 0], color=color, marker="o", s=116)
        marker = {"soft-precise": "^", "hard-imprecise": "o", "aborted": "x"}[met["cat"]]
        if met["cat"] == "hard-imprecise":
            ax3d.scatter(X[0, -1], X[1, -1], -X[2, -1], facecolors="none", edgecolors=color,
                         marker=marker, s=183, linewidths=2.6)
        else:
            ax3d.scatter(X[0, -1], X[1, -1], -X[2, -1], color=color, marker=marker, s=183)
        if not target_drawn:
            xt = d.x_t[:3, :N]
            draw_landing_corridor(ax3d, xt[0], xt[1], xt[2])
            target_drawn = True
    ax3d.set_xlabel(r"$\,^\mathcal{I}x_\mathrm{b}$ [m]", labelpad=40, fontsize=fontsize)
    ax3d.set_ylabel(r"$\,^\mathcal{I}y_\mathrm{b}$ [m]", labelpad=40, fontsize=fontsize)
    ax3d.set_zlabel(r"$\,^\mathcal{I}z_\mathrm{b}$ [m]", labelpad=18, fontsize=fontsize)
    ax3d.locator_params(axis="x", nbins=4)
    ax3d.locator_params(axis="y", nbins=4)
    ax3d.locator_params(axis="z", nbins=4)
    ax3d.tick_params(pad=2, labelsize=ticksize)
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
    ax.set_xlabel("Only landed runs shown (full outcomes in (a))", fontsize=24, color="red",
                  labelpad=20)


def _draw_fov_3d(ax, fontsize=16, tick_fontsize=14, zoom=1.18, elong=1.8):
    """3-D visibility panel, Case 5, all 5 controllers -- x axis is time t
    (2026-09-16: swapped from z, so the long axis is the one that actually
    spans a wide range; the box no longer needs to stay cube-shaped), y/z
    axes are the FoV-normalized marker centre [Phi^-1 {}^C r-tilde]_y,
    [Phi^-1 {}^C r-tilde]_x (each in [-1,1] iff inside the buffered
    visibility set S_vis = {r-tilde : ||Phi^-1 r-tilde||_inf <= 1}).
    Replaces the 2026-09-16 2-D 'min_k h_k(t)' line chart per explicit user
    objection ("this is not how CBF works"): a single min-combined scalar
    line implies one scalar CBF was analyzed, when the QP/theorem impose
    h_x>=0 and h_y>=0 as two separate per-axis constraints. Plotting the
    normalized position directly (not a barrier value) makes both axes'
    constraint satisfaction independently visible from the same curve, with
    no min() anywhere in the plotted quantity itself.

    The safe box [-1,1]x[-1,1] (in y,z) is drawn as a wireframe prism
    extruded along the time axis (4 edges + t=0/t=t_max rims) -- a
    controller's curve exiting the box at some t is exactly a visibility
    violation at that instant, on whichever axis it crosses."""
    t_max = 0.0
    for name in CTRLS:
        run = RUNS[(CASE5, name)]
        N = METRICS[(CASE5, name)]["N"]
        rx, ry, _ = _fov_margin(run, N)      # may trim further than N -- see docstring
        nx = rx / PHI_MAX[0]                 # [Phi^-1 r-tilde]_x
        ny = ry / PHI_MAX[1]                 # [Phi^-1 r-tilde]_y
        t = run.data.tRange[:len(nx)]
        t_max = max(t_max, float(t[-1]) if len(t) else 0.0)
        # lw bumped from 2.2: at print scale (this panel's curves get shrunk far more
        # than the other panels', since FIG_W=19in vs a ~3.5in column), a steeply-dipping
        # segment (verified continuous -- no NaNs, uniform 0.01s dt) can visually alias
        # into a dotted/beaded look at the thinner effective stroke width; 3.2 keeps it
        # solid without visibly thickening the shallower parts of the curve.
        ax.plot(t, ny, nx, color=CTRL_COLORS[name], lw=5.0, label=CTRL_DISPLAY[name], zorder=3)

    # Safe-box wireframe: 4 edges (t=0 to t=t_max) + the two end rims.
    corners = [(1, 1), (-1, 1), (-1, -1), (1, -1), (1, 1)]  # (ny, nx) pairs
    cy = [c[0] for c in corners]; cx = [c[1] for c in corners]
    ax.plot([0] * 5, cy, cx, color="k", lw=1.3, ls="--", alpha=0.6, zorder=1)
    ax.plot([t_max] * 5, cy, cx, color="k", lw=1.3, ls="--", alpha=0.6, zorder=1)
    # The 4 long edges connecting the t=0/t=t_max rims used to be drawn here as dotted
    # lines, but mplot3d's per-artist (not per-fragment) depth sorting made them render
    # as visibly broken/discontinuous once the box's proportions changed -- removed
    # rather than patched, since the two dashed rims already mark the safe box's extent.

    ax.set_xlabel(r"$t$ [s]", fontsize=fontsize, labelpad=38)
    ax.set_ylabel(r"$\,^\mathcal{C}\tilde{r}_y$", fontsize=fontsize, labelpad=14)
    ax.set_zlabel(r"$\,^\mathcal{C}\tilde{r}_x$", fontsize=fontsize, labelpad=14)
    ax.tick_params(pad=1, labelsize=tick_fontsize)
    ax.locator_params(axis="x", nbins=4)
    ax.locator_params(axis="y", nbins=4)
    ax.locator_params(axis="z", nbins=4)
    ax.view_init(elev=22, azim=-58)
    # Elongate the time axis explicitly -- matplotlib doesn't stretch a 3-D
    # box just because the x data range is wider; box_aspect is the actual
    # rendered-shape control. elong:1:1 keeps y/z (both [-1,1]) square to
    # each other while giving time some visual length.
    ax.set_box_aspect((elong, 1, 1), zoom=zoom)


# ============================================================================
# Figure 1: comparison_qual_combined.pdf (2x2) -- the main comparison
#   figure, per the recommended set (2026-09-11): (a) outcome heatmap --
#   overall comparison, which controller lands/aborts/is hard/soft; (b) the
#   Circular (Case 5) 3D trajectory -- a representative qualitative case
#   study, the richest/most demanding case; (c) FoV margin, Case 5 -- WHY
#   baselines fail, not just that they do; (d) relative touchdown energy --
#   soft-landing quality as a severity measure, not a binary pass/fail.
# ============================================================================
#   Layout: every panel is placed directly by an explicit vertical "stack" of
#   inch-height constants (title space -> axes -> gap -> title space -> axes
#   -> ... -> legend), with the figure's total height DERIVED as the sum of
#   that stack. Column X-positions and every axes box below are pinned to
#   fixed ABSOLUTE INCH values (comments say which), converted to figure
#   fractions via division by FIG_W/FIG_H at the point of use -- so changing
#   FIG_W/FIG_H (e.g. trimming canvas margin) never rescales a box, it only
#   changes how much blank margin surrounds the fixed-size content.
#
#   Fontsize note: included at \columnwidth same as multi_init's
#   Circular_combined.pdf (figsize width 10.5in, title/label fontsize 20) --
#   this canvas is narrower, so the same fontsize numbers would print
#   smaller here; PANEL_TITLE_FS etc. below are pre-scaled to land at
#   multi_init's effective on-page size.
PANEL_TITLE_FS = 34
TITLE_X_SHIFT  = 0.8   # inches -- shifts all four panel subtitles left (explicit user request),
                        # nothing else (axes/plots/legend/suptitle positions are untouched)
FIG_W = 19.0    # canvas width -- bounded by the suptitle (fontsize 55, one line) needing
                # this much room; every plotted panel fits comfortably inside ~18.6in.

# Column X-positions (left, width), in inches -- from a plain 2-column
# gridspec (width_ratios=[1.0, 1.25], wspace=0.28, left=0.13, right=0.98)
# at a 21.5in reference width, converted to absolute inches once here.
COL0_X0, COL0_W = 2.795, 7.12467
COL1_X0, COL1_W = 12.164055, 8.905945
# (b)'s box: shifted/narrowed from column 1 to open a gap from (a) on its
# left (a 2-D bar-chart panel doesn't need column 1's full width) while
# holding its right edge fixed; capped short of aligning box edges because
# (a)'s S/H/A legend (legend_ncol=3) extends past ax_hm's own right edge.
AX3D_X0, AX3D_W = 12.084055, 6.540945

# Vertical stack, top to bottom, in inches from the figure's top edge.
# Row 1 has UNEQUAL heights by design: (a) and (b) are both 2-D panels sized
# to match each other; row 2's (c)/(d) are square 3-D boxes sized
# independently below (ROW2_AX_H).
TOP_MARGIN    = 1.30     # blank canvas above the row-1 titles
TITLE_H       = 0.64     # space reserved above each row for its subtitle (scaled w/ PANEL_TITLE_FS)
HM_AX_H       = 3.80     # panel (a) axes height
HM_LEGEND_H   = 0.75     # room for (a)'s S/H/A legend, which hangs below its axes box
ROW1_B_H      = 3.80     # panel (b) axes height, matched to (a)'s
ROW_GAP       = 0.75     # gap between row 1's lowest content and row 2's box top -- (c)/(d) use
                          # the INSET set_title(y=0.95) convention (see _c_p/_d_p below), which
                          # sits inside the box rather than in a reserved strip above it, so this
                          # only needs to clear row 1's legend/footnote text, not a title too.
ROW2_BOX_W    = 6.540945  # panel (c)'s ORIGINAL square-box side -- still used below for every
                          # WIDTH-related calc (AX_M_W, and (d)'s width/X0 centering), since only
                          # HEIGHT changes here.
ROW2_H_INCREASE = 1.0     # inches -- (c) grows by this on BOTH height and width (square, explicit
                          # user request, so its width also becomes ROW2_BOX_W + increase); (d)
                          # grows by the SAME amount on height ONLY, width held fixed (explicit
                          # user request) -- so (d)'s box is no longer square.
ROW2_C_H      = ROW2_BOX_W + ROW2_H_INCREASE  # (c)'s box height AND width (aspect preserved)
ROW2_D_H      = ROW2_BOX_W + ROW2_H_INCREASE  # (d)'s box HEIGHT only; its width stays ROW2_BOX_W-derived
ROW2_XLABEL_H = 0.40     # room for (c)/(d)'s x-tick labels + xlabel, which sit below the row-2 boxes
LEGEND_GAP    = 0.05     # gap between (c)/(d)'s x-labels and the shared bottom controller legend
LEGEND_H      = 0.72     # provisional -- corrected below from the actual render
BOTTOM_MARGIN = 0.12

row1_top    = TOP_MARGIN + TITLE_H
ax3d_top    = row1_top
row1_bottom = max(row1_top + HM_AX_H + HM_LEGEND_H, ax3d_top + ROW1_B_H)
row2_top    = row1_bottom + ROW_GAP
row2_bottom = row2_top + max(ROW2_C_H, ROW2_D_H)
legend_top  = row2_bottom + ROW2_XLABEL_H + LEGEND_GAP
FIG_H       = legend_top + LEGEND_H + BOTTOM_MARGIN


def _y0_frac(top_in, height_in):
    """(distance-from-top, height), both inches -> matplotlib's bottom-up y0 fraction."""
    return (FIG_H - top_in - height_in) / FIG_H


def _panel_title(ax, text):
    """Panel subtitle centered above ax, a fixed 0.10in above its top edge --
    shared by every RESERVED-STRIP-style panel ((a)/(b); (c)/(d) use the
    inset convention instead, see _inset_title). Uses
    get_position(original=True): Axes3D silently shrinks its box to
    preserve aspect and reports that shrunk "active" box from plain
    get_position(), which would throw off alignment between panels whose
    assigned height/width ratios differ; original=True returns the box we
    actually assigned, for both 2-D and 3-D axes alike."""
    p = ax.get_position(original=True)
    ax.figure.text((p.x0 + p.x1) / 2 - TITLE_X_SHIFT / FIG_W, p.y1 + 0.10 / FIG_H, text,
                    ha="center", va="bottom", fontsize=PANEL_TITLE_FS)


def _inset_title(ax, text, x_ref_center):
    """Panel subtitle for a 3-D panel, INSET near its box's own top edge
    rather than in a reserved strip above it (matches make_comparison_plots.py's
    3-D panel convention; needs less vertical margin than _panel_title,
    letting row 2 sit closer to row 1). Placed via fig.text at figure-
    fraction coordinates rather than ax.set_title(x=,y=): set_title's x/y
    are fractions of get_position(), and for Axes3D that's the "active" box
    (see _panel_title's docstring), which moves independently of the
    assigned box whenever box_aspect/zoom changes -- fig.text off
    get_position(original=True) is immune to that. x_ref_center is the
    FIGURE-fraction x to center on (so (c)/(d)'s titles can be laterally
    aligned with (a)/(b) above them, even though the box pairs have
    different widths/positions)."""
    p = ax.get_position(original=True)
    ax.figure.text(x_ref_center - TITLE_X_SHIFT / FIG_W, p.y1 - 0.05 * (p.y1 - p.y0), text,
                    fontsize=PANEL_TITLE_FS, ha="center", va="top")


fig = plt.figure(figsize=(FIG_W, FIG_H))

ax_hm = fig.add_axes([COL0_X0 / FIG_W, _y0_frac(row1_top, HM_AX_H), COL0_W / FIG_W, HM_AX_H / FIG_H])
ax3d  = fig.add_axes([AX3D_X0 / FIG_W, _y0_frac(ax3d_top, ROW1_B_H), AX3D_W / FIG_W, ROW1_B_H / FIG_H])

# (c)/(d): square 3-D boxes (side ROW2_AX_H), centred under columns 0/1 then
# shifted left by ROW2_SHIFT_LEFT -- COL0_W/COL1_W are narrower than what
# plain centring would want once paired with the square box, so centring
# alone leaves a visible empty margin on the left and clips (d)'s right-side
# tick/axis labels against the figure edge; shifting both boxes left the
# same amount fixes both.
ROW2_SHIFT_LEFT = 2.6125  # inches -- bumped by 1.0in (explicit user request) to move (c)/(d) left
AX_M_W = ROW2_C_H  # square: width grows with height
AX_M_X0 = COL0_X0 + (COL0_W - AX_M_W) / 2.0 - ROW2_SHIFT_LEFT
ax_m  = fig.add_axes([AX_M_X0 / FIG_W, _y0_frac(row2_top, ROW2_C_H), AX_M_W / FIG_W, ROW2_C_H / FIG_H],
                      projection="3d")

# (d): same square-box centring as (c), narrowed (2.15in) and shifted
# further left (2.99in net) to clear (a)'s legend/etc -- then WIDENED
# rightward only (left edge fixed) to fill the empty space up to the
# figure's right margin (explicit user request): its RENDERED 3-D content
# (not just its assigned box) overspills the box by ~0.37in on the left
# (a box_aspect/zoom side effect), so a bit of that leftward shift is given
# back as clearance from (c) once widened; _D_ELONG/_D_ZOOM below are tuned
# to fill the resulting wider box while keeping the same rendered HEIGHT as
# (c).
_ax_ke_center = COL1_X0 + COL1_W / 2.0 - ROW2_SHIFT_LEFT
_ax_ke_w_narrow = ROW2_BOX_W - 2.15
AX_KE_X0 = _ax_ke_center - _ax_ke_w_narrow / 2.0 - 2.99 - 0.5  # left edge, held fixed as width
                          # grows; extra 0.5in shift added when (d)'s height was increased --
                          # the taller box's changed 3-D perspective pushed the z-label right
                          # enough to clip against the figure edge, so this recovers margin
AX_KE_W = 8.68  # unchanged (explicit user request: (d)'s height grows, its width does not)
ax_ke = fig.add_axes([AX_KE_X0 / FIG_W, _y0_frac(row2_top, ROW2_D_H), AX_KE_W / FIG_W, ROW2_D_H / FIG_H],
                      projection="3d")

# Category legend dropped here (redundant with the caption's S/H/A key and
# collided with the shared bottom controller-color legend); kept only on the
# standalone comparison_outcome_heatmap.pdf.
_draw_heatmap(ax_hm, cell_fontsize=30, tick_fontsize=27,
              show_legend=True, legend_ncol=3, legend_anchor="auto-left", legend_fontsize=27,
              show_title=False)
_draw_energy(ax3d, fontsize=32, tick_fontsize=27)      # (b): Relative Touchdown Energy
_draw_3d(ax_m, fontsize=32, ticksize=27)               # (c): Landing Trajectories
# _D_ELONG (box_aspect's time-axis:y:z ratio) and _D_ZOOM were both tuned by
# rendering and comparing (d)'s rendered height against (c)'s: (i) matplotlib's
# Line3D.get_window_extent() is NOT reliable in this matplotlib version (huge
# nonsense extents, insensitive to box_aspect/zoom) so it can't be used to
# measure this directly; (ii) proj3d.proj_transform + ax.transData applied to
# the safe-box's fixed corner coordinates IS reliable and was used instead to
# solve for the _D_ZOOM that makes (d)'s projected vertical span match (c)'s.
_D_ELONG = 2.2   # reduced from 2.377664336126142 (explicit user request)
_D_ZOOM = 1.215  # re-tuned (same proj3d.proj_transform + ax.transData calibration method as
                 # before) to hold (d)'s rendered height at (c)'s after the elongation change
_draw_fov_3d(ax_ke, fontsize=32, tick_fontsize=27, zoom=_D_ZOOM, elong=_D_ELONG)  # (d): Marker Visibility

_panel_title(ax_hm, "(a) Closed-Loop Outcome")
_panel_title(ax3d, "(b) Relative Touchdown Energy")
_inset_title(ax_m, "(c) Landing Trajectories, Case 5", (COL0_X0 + COL0_W / 2.0) / FIG_W)
_inset_title(ax_ke, "(d) Marker Visibility, Case 5", (AX3D_X0 + AX3D_W / 2.0) / FIG_W)

# Legend anchored just below row 2 (upper-center at legend_top), not at the
# absolute figure bottom -- ties its position to the content above it
# directly, instead of relying on a separately-sized blank canvas below.
# Single row (ncol=5): with the short "Proposed"/"Baseline A-D" labels this
# fits the canvas width comfortably. Handles/labels come from ax_m (Landing
# Trajectories), the only row-2 panel that plots+labels all 5 CTRLS --
# _draw_energy only labels REACHED_CTRLS (2 of 5).
handles, labels = ax_m.get_legend_handles_labels()
ctrl_legend = fig.legend(handles, labels, loc="upper center", ncol=5,
           bbox_to_anchor=(0.5, _y0_frac(legend_top, 0.0)),
           frameon=False, fontsize=29, handlelength=1.6, columnspacing=1.6, handletextpad=0.5)
for line in ctrl_legend.get_lines():
    line.set_linewidth(3.0)

fig.suptitle("Closed-Loop Comparison with Baseline Controllers",
             fontsize=55, y=0.985)
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
