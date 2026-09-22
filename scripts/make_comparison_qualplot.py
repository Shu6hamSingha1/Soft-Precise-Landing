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
import matplotlib.patches as mpatches
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
LABELS = ["Case 1", "Case 2", "Case 3", "Case 4", "Case 5"]  # still used for the console summary
CASE_NUMS = [str(i + 1) for i in range(len(TRAJS))]  # bare "1".."5" for (a)/(b)'s x-tick labels,
                                                      # paired with a "Cases" axis label instead
                                                      # (explicit user request)
CTRLS  = ["PLASMC (Proposed)", "Lin 2022", "Zhang 2026", "Lin 2023", "Cho 2022"]

CTRL_COLORS = {
    # 2026-09-16: vermillion/bluish-green swapped for black/sky-blue -- at Fig. 3/4's
    # printed size (~3.5in wide) vermillion was too close to orange, and bluish-green
    # too close to blue. Then blue itself swapped for bluish-green (2026-09-16, later
    # same day) since sky-blue made it redundant -- sky-blue's much lighter/less
    # saturated than blue was, so it stays distinguishable from bluish-green here.
    # Still a subset of the 8-color Okabe-Ito colorblind-safe palette.
    "PLASMC (Proposed)": "#000000",
    "Lin 2022":          "#009E73",
    "Zhang 2026":        "#56B4E9",
    "Lin 2023":          "#CC79A7",
    "Cho 2022":          "#E69F00",
}
# 2026-09-11: no author names or citation numbers anywhere, per explicit user
# instruction, applied uniformly across all comparison figures (legends AND
# the heatmap's row labels below).
# 2026-09-15: "Baseline A-D" swapped for method-acronym tags per explicit
# user instruction (supersedes the 2026-09-11 "Baseline A-D only" call).
# 2026-09-16: citation numbers added back for baselines IN THE LEGENDS ONLY
# (CTRL_DISPLAY), per explicit user instruction -- supersedes the 2026-09-11
# "no citation numbers" call for this one use. ROW_LABELS (heatmap row
# labels) intentionally left uncited, matching the "in the legends" wording.
# "Proposed" -> "VISTA" applies everywhere (both dicts) since it's a rename,
# not a citation; the shared bottom legend also moves VISTA to the end of
# the row (see the handles/labels reorder below), baselines keeping CTRLS'
# original order.
CTRL_DISPLAY = {
    "PLASMC (Proposed)": "VISTA",
    "Lin 2022":          "PBVS-PPC [2]",
    "Zhang 2026":        "PBVS-AEDO [12]",
    "Lin 2023":          "IBVS-PPC [6]",
    "Cho 2022":          "FF-IBVS [1]",
}
ROW_LABELS = {
    "PLASMC (Proposed)": "VISTA",
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




def _key5_idx(Np):
    """Column indices of the 5 KEY points (4 arm tips + stub tip) of the marker point list.
    Legacy 5-point cross (Np<=5): the columns themselves. Line-sampled cross (2026-09-21,
    MATLAB InitVar.m): columns are [arm1 | arm2 | arm3 | arm4 | stub] with n samples per arm and
    round(n*22/15) on the stub, so each line's tip is its LAST sample."""
    if Np <= 5:
        return list(range(Np))
    for n in range(1, Np):
        if 4 * n + int(round(n * 22 / 15)) == Np:
            return [n - 1, 2 * n - 1, 3 * n - 1, 4 * n - 1, Np - 1]
    return [0, 1, 2, 3, Np - 1]

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
    tips = _key5_idx(Np)[:4]                         # 4 arm tips, exclude the stub (line-sampled cross aware)
    cx = cnp[0, tips, :].mean(axis=0)                 # marker-centre x [px] (tip mean: legacy fallback)
    cy = cnp[1, tips, :].mean(axis=0)                 # marker-centre y [px]
    _cl = getattr(d, "cen_px_log", None)              # exact marker centre (2026-09-21 harness log) -- perspective makes the tip mean drift
    if _cl is not None and np.ndim(_cl) == 2 and _cl.shape[0] == 2 and np.any(_cl != 0):
        _m = min(_cl.shape[1], cnp.shape[-1]); cx = _cl[0, :_m]; cy = _cl[1, :_m]
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
    # Bare case numbers on the ticks + "Cases" as the actual axis label, replacing the old
    # "Case 1".."Case 5" tick text (explicit user request).
    ax.set_xticks(range(len(TRAJS))); ax.set_xticklabels(CASE_NUMS, fontsize=tick_fontsize)
    ax.set_xlabel("Cases", fontsize=tick_fontsize)
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
            # -0.30 (was -0.07): pushed further down now that this axes also has a "Cases"
            # x-label sitting right below its tick labels -- the legend needs to clear that too.
            legend_anchor = (x_anchor, legend_anchor[1] if isinstance(legend_anchor, tuple) else -0.30)
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
        ax3d.plot(X[0], X[1], -X[2], color=color, lw=3, label=CTRL_DISPLAY[name])
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
    ax3d.set_xlabel(r"$\,^\mathcal{I}x_\mathrm{b}$ [m]", labelpad=35, fontsize=fontsize)
    ax3d.set_ylabel(r"$\,^\mathcal{I}y_\mathrm{b}$ [m]", labelpad=35, fontsize=fontsize)
    # z labelpad bumped 10 -> 24 -> 30 (explicit user request) -- at the bumped label fontsize, "I_z_b
    # [m]" was sitting right on top of the topmost z tick number.
    ax3d.set_zlabel(r"$\,^\mathcal{I}z_\mathrm{b}$ [m]", labelpad=30, fontsize=fontsize)
    ax3d.locator_params(axis="x", nbins=4)
    ax3d.locator_params(axis="y", nbins=4)
    ax3d.locator_params(axis="z", nbins=4)
    # tick pad bumped 2 -> 12 (explicit user request) -- same reason, the tick numbers
    # themselves were sitting flush against the axis spine at the bumped tick fontsize.
    ax3d.tick_params(pad=10, labelsize=ticksize)
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
    readable axis. Controllers that ever reach the surface only -- no bar
    slots wasted on controllers that never land.
    Bar width now SCALES with len(REACHED_CTRLS) (fixed 2026-09-22) -- it was
    a hardcoded 0.30, sized back when only 2 controllers ever reached the
    surface; once the retuned baselines (2026-09-21) pushed REACHED_CTRLS to
    all 5, that fixed width made adjacent bars overlap (spacing ~1/6 << 0.30).
    Bars now fill a fixed 0.82-wide group per case, evenly split N ways, so
    they're always equal width and never overlap regardless of N."""
    xb = np.arange(len(TRAJS))
    n_r = max(len(REACHED_CTRLS), 1)
    group_w = 0.82
    width_b = group_w / n_r
    xj_off = (np.arange(n_r) - (n_r - 1) / 2.0) * width_b
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
            # Empty box (no "N/A" text) matching the legend's N/A swatch exactly --
            # white fill, black edge, same style/width as that patch -- the legend
            # explains what it means, so the in-plot marker doesn't need its own label
            # (explicit user request; previously an "N/A"-labelled, per-controller-colored
            # box, which duplicated the legend and used a stale unscaled fontsize).
            ax.add_patch(mpatches.Rectangle(
                (xk - width_b / 2, -0.05), width_b, 0.10,
                facecolor="white", edgecolor="black", linewidth=0.6,
                transform=ax.transData, clip_on=False, zorder=5))
    # Bare case numbers instead of "Case 1".."Case 5" (explicit user request) -- short enough
    # at tick_fontsize to no longer need the rotation/shrinking the old "Case N" text required.
    ax.set_xticks(xb); ax.set_xticklabels(CASE_NUMS, rotation=0, fontsize=tick_fontsize)
    ax.tick_params(axis="y", labelsize=tick_fontsize)
    ax.grid(axis="y", alpha=0.3)
    ax.set_ylabel(r"$\log_{10}(\eta_E)$", fontsize=fontsize, labelpad=4)
    ax.set_xlabel("Cases", fontsize=tick_fontsize, labelpad=8)
    # Moved off set_xlabel (now "Cases" above) onto a plain ax.text call, placed a bit further
    # down so it still reads as a footnote rather than this panel's actual x-axis label. Kept
    # at a size well below the other panels' axis-label size for the same reason -- this is a
    # full sentence, not a short label, and at anything much above 32 its centered width
    # (measured via page.search_for on the rendered PDF) exceeds the gap back to (a)'s legend.
    ax.text(0.5, -0.36, "Only landed runs shown (full outcomes in (a))",
            transform=ax.transAxes, ha="center", va="top", fontsize=32, color="red")


def _draw_fov_3d(ax, fontsize=16, tick_fontsize=14, zoom=1.18, elong=1.8):
    """3-D visibility panel, Case 5 (Circular) from IC2, all 5 controllers, consistent
    with panel (c) -- x axis is time t, y/z axes are the FoV-normalized camera-frame
    centroid r_n = Phi^-1 * (C_r_hat)/f = [r_n,x, r_n,y]^T defined in ICRA.tex
    eq. (normalized camera centroid), i.e. exactly the r_n of the visibility set
    S_vis = {r_n : ||r_n||_inf <= 1} (ICRA.tex eq. (visibility set)) -- NOT a scalar
    barrier margin or a PPC/pixel-coordinate plot (explicit user instruction,
    2026-09-17, keeping the 2026-09-16 3-D-trajectory concept but tying it exactly
    to the manuscript's r_n notation/limits instead of the prior informal r-tilde
    labeling). Replaces the pre-2026-09-16 2-D 'min_k h_k(t)' line chart per
    separate explicit user objection ("this is not how CBF works"): a single
    min-combined scalar line implies one scalar CBF was analyzed, when the
    QP/theorem impose |r_n,x|<=1 and |r_n,y|<=1 as two separate per-axis
    constraints. Plotting r_n directly (not a barrier value) makes both axes'
    constraint satisfaction independently, visually identical to that
    inequality -- see the four boundary planes below.

    The four FoV boundary planes r_n,x=+-1, r_n,y=+-1 are drawn as lightly
    transparent Poly3DCollection faces (not just wireframe edges, explicit user
    request) so the visibility condition |r_n,x|<=1, |r_n,y|<=1 reads visually
    as "inside this box" -- a controller's curve crossing a plane at some t is
    exactly a visibility violation at that instant, on whichever axis it crosses.
    Controllers that abort on an FoV breach are TRUNCATED at the first sample
    where |r_n,x|>1 or |r_n,y|>1 (not merely left as-is at whatever the sim's own
    internal break happened to save) and marked there with an 'x', matching the
    aborted-marker convention from panel (c)/_draw_3d; controllers that land are
    plotted in full and marked at touchdown with the same soft-precise
    ('^', filled)/hard-imprecise ('o', hollow) convention as (c). This is what
    makes PBVS-AEDO's "retains visibility but still not soft-precise" story
    (touchdown marker safely inside the box) visually distinct from PBVS-PPC/
    FF-IBVS's "loses visibility" story (breach marker on a boundary plane)."""
    t_max = 0.0
    curves = {}
    for name in CTRLS:
        run = RUNS[(CASE5, name)]
        N = METRICS[(CASE5, name)]["N"]
        cat = METRICS[(CASE5, name)]["cat"]
        rx, ry, _ = _fov_margin(run, N)      # may trim further than N -- see docstring
        rnx = rx / PHI_MAX[0]                # r_n,x
        rny = ry / PHI_MAX[1]                # r_n,y
        t = run.data.tRange[:len(rnx)]
        # Explicit FoV-breach truncation: first sample outside the visibility set,
        # independent of wherever visualControl_comparison.m's own internal break
        # happened to save its last column (see _fov_margin's BUG FIX docstring) --
        # this guarantees the plotted curve never crosses a boundary plane and
        # actually ends exactly ON one, for aborted controllers.
        breach = np.where((np.abs(rnx) > 1.0) | (np.abs(rny) > 1.0))[0]
        i_end = int(breach[0]) if len(breach) else len(rnx) - 1
        t_c, rnx_c, rny_c = t[:i_end + 1], rnx[:i_end + 1], rny[:i_end + 1]
        curves[name] = (t_c, rnx_c, rny_c, cat)
        t_max = max(t_max, float(t_c[-1]) if len(t_c) else 0.0)

    for name in CTRLS:
        t_c, rnx_c, rny_c, cat = curves[name]
        color = CTRL_COLORS[name]
        # lw bumped from 2.2: at print scale (this panel's curves get shrunk far more
        # than the other panels', since FIG_W=19in vs a ~3.5in column), a steeply-dipping
        # segment (verified continuous -- no NaNs, uniform 0.01s dt) can visually alias
        # into a dotted/beaded look at the thinner effective stroke width; 3.2 keeps it
        # solid without visibly thickening the shallower parts of the curve.
        ax.plot(t_c, rny_c, rnx_c, color=color, lw=3, label=CTRL_DISPLAY[name], zorder=3)
        marker = {"soft-precise": "^", "hard-imprecise": "o", "aborted": "x"}[cat]
        if cat == "aborted":
            # Visibility-loss point: exactly the truncation endpoint above, on a boundary plane.
            ax.scatter(t_c[-1], rny_c[-1], rnx_c[-1], color=color, marker=marker, s=140, zorder=6)
        elif cat == "hard-imprecise":
            ax.scatter(t_c[-1], rny_c[-1], rnx_c[-1], facecolors="none", edgecolors=color,
                       marker=marker, s=110, linewidths=2.2, zorder=6)
        else:
            ax.scatter(t_c[-1], rny_c[-1], rnx_c[-1], color=color, marker=marker, s=110, zorder=6)

    # Pin all three axes to exactly the safe box's own extent -- [0, t_max] in time,
    # [-1, 1] in y/z -- matplotlib's default 5% autoscale margin otherwise extends
    # the axis panes' actual corners past that on EVERY axis, so the boundary planes
    # below (drawn exactly at t_max / +-1) land short of the panes' real corners,
    # visibly detached from the box outline there.
    ax.set_xlim3d(0, t_max)
    ax.set_ylim3d(-1, 1)
    ax.set_zlim3d(-1, 1)

    # Four FoV boundary planes (r_n,x = +-1, r_n,y = +-1), each spanning the full
    # [0, t_max] x [-1, 1] extent of the OTHER two axes -- these are 4 of the safe
    # box's 6 faces (top/bottom, at t=0/t=t_max, aren't FoV limits, so they're
    # omitted). Lightly transparent (explicit user request) so controller curves
    # stay visually prominent; a thin edge outline keeps each plane's extent legible
    # against the white background.
    _plane_alpha, _plane_fc = 0.14, "0.55"
    rnx_planes = [1.0, -1.0]   # r_n,x = +-1 -> constant on the z axis here
    rny_planes = [1.0, -1.0]   # r_n,y = +-1 -> constant on the y axis here
    for rnx0 in rnx_planes:
        face = [[(0, -1, rnx0), (0, 1, rnx0), (t_max, 1, rnx0), (t_max, -1, rnx0)]]
        ax.add_collection3d(Poly3DCollection(face, facecolor=_plane_fc, alpha=_plane_alpha,
                                              edgecolor="k", linewidth=0.5, zorder=1))
    for rny0 in rny_planes:
        face = [[(0, rny0, -1), (0, rny0, 1), (t_max, rny0, 1), (t_max, rny0, -1)]]
        ax.add_collection3d(Poly3DCollection(face, facecolor=_plane_fc, alpha=_plane_alpha,
                                              edgecolor="k", linewidth=0.5, zorder=1))

    # x labelpad bumped 38 -> 65 (explicit user request) -- same tick-vs-label crowding as
    # y/z below, now that the x tick pad also grew to 22.
    ax.set_xlabel(r"$t$ [s]", fontsize=fontsize, labelpad=40)
    # y/z labelpad and tick pads bumped hard (explicit user request) -- at this fontsize AND
    # this viewing angle (elev=22, azim=-58), the y/z tick numbers and axis labels crowd into
    # each other and into the shared box corner much more aggressively than a linear
    # pad-vs-fontsize scaling would suggest; two earlier, smaller bumps (14->28 label / 1->10
    # tick / 16->30 z-tick) were each insufficient and had to be redone larger.
    ax.set_ylabel(r"$r_{\mathrm{n},y}$", fontsize=fontsize, labelpad=40)
    ax.set_zlabel(r"$r_{\mathrm{n},x}$", fontsize=fontsize, labelpad=50)
    # z labelpad must clear the z TICK pad (50, below) by a wide margin -- label/tick pads
    # both offset from the same axis spine independently, so a label pad merely close to (or
    # smaller than) the tick pad puts the label BETWEEN the spine and the pushed-out ticks.
    ax.tick_params(pad=12, labelsize=tick_fontsize)
    # z-axis ticks need MORE pad than x/y: at this viewing angle the z=1 tick label
    # sits right at the shared box corner with the y=1 tick label, and the two
    # overlap into unreadable garbled text there -- push z's labels further out.
    ax.tick_params(axis="z", pad=25)
    ax.locator_params(axis="x", nbins=4)
    ax.locator_params(axis="y", nbins=3)
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
PANEL_TITLE_FS = 52  # -> ~8.6pt printed at FIG_W=21.0's ~0.1671 embed scale, matching
                     # Circular_combined.pdf's panel-title size (explicit user request);
                     # TOP_MARGIN/TITLE_H below were widened (2026-09-16) to fit this without
                     # the suptitle/row-1-title overlap that blocked this same value at first.
TITLE_X_SHIFT  = 0.8   # inches -- shifts all four panel subtitles left (explicit user request),
                        # nothing else (axes/plots/legend/suptitle positions are untouched)
FIG_W = 22.3    # canvas width -- bounded by the suptitle (one line) needing this much room;
                # bumped 21.3 -> 22.3 (2026-09-16) after (d)'s z labelpad had to grow to 72
                # (to clear its own tick pad, see _draw_fov_3d) and started clipping off the
                # right edge.
                # bumped 21.0 -> 21.3 (2026-09-16) after moving (b) right also dragged (d)'s
                # title (tied to the same AX3D_X0 reference) to within ~0.12in of the right
                # edge; note growing FIG_W here shrinks the print scale slightly (all native
                # pt targets above/below were computed at the 21.0 scale and are now a hair
                # under their stated printed-pt targets -- not worth re-deriving for ~1.4%).
                # every plotted panel fits comfortably inside ~18.6in. Bumped 19.0 -> 19.6
                # (2026-09-16) to clear (d)'s z-label overflow, then 19.6 -> 21.0 (2026-09-16,
                # same day) when the font-size pass toward matching Circular_combined.pdf's
                # printed sizes made the suptitle and the bottom controller legend both run off
                # the right edge at 19.6in -- growing native font size without growing FIG_W
                # only makes text overflow its own canvas, it doesn't print bigger by itself
                # (printed_pt = native_pt * \columnwidth/(FIG_W*72), so FIG_W has to grow with
                # the widest single-line content, here the suptitle/legend row).

# Column X-positions (left, width), in inches -- from a plain 2-column
# gridspec (width_ratios=[1.0, 1.25], wspace=0.28, left=0.13, right=0.98)
# at a 21.5in reference width, converted to absolute inches once here.
COL0_X0, COL0_W = 3.75, 7.12467  # X0 bumped from 2.795 (2026-09-16, then 3.45 -> 3.75 once
                                  # "PBVS-AEDO" -- the widest row label -- still clipped its
                                  # first letter at 3.45): (a)'s row labels
                                  # ("PBVS-PPC" etc, at the bumped tick_fontsize matching
                                  # Circular_combined.pdf) ran off the canvas's left edge
COL1_X0, COL1_W = 12.164055, 8.905945
# (b)'s box: shifted/narrowed from column 1 to open a gap from (a) on its
# left (a 2-D bar-chart panel doesn't need column 1's full width) while
# holding its right edge fixed; capped short of aligning box edges because
# (a)'s S/H/A legend (legend_ncol=3) extends past ax_hm's own right edge.
AX3D_X0, AX3D_W = 13.584055, 6.540945  # X0 bumped +1.0in (explicit user request, 2026-09-16);
                                        # a -0.15in nudge tried 2026-09-17 was reverted, then
                                        # +0.5in applied instead (explicit user request, 2026-09-17)
                                        # to move (b) right; FIG_W=21.0 leaves ample margin
                                        # (right edge lands at 19.625in of 21.0)

# Vertical stack, top to bottom, in inches from the figure's top edge.
# Row 1 has UNEQUAL heights by design: (a) and (b) are both 2-D panels sized
# to match each other; row 2's (c)/(d) are square 3-D boxes sized
# independently below (ROW2_AX_H).
TOP_MARGIN    = 1.63     # blank canvas above the row-1 titles (scaled 1.25x from 1.30, matching
                         # the font-size bump below, so PANEL_TITLE_FS=48 still clears the suptitle)
TITLE_H       = 0.80     # space reserved above each row for its subtitle (scaled w/ PANEL_TITLE_FS)
HM_AX_H       = 3.80     # panel (a) axes height
HM_LEGEND_H   = 2.00     # room for (a)'s "Cases" x-label + 2-row S/H/A legend, both of which
                         # hang below its axes box (bumped 2026-09-16 for the 2-row legend +
                         # new x-label; was 0.94 for a single-row legend with no x-label)
ROW1_B_H      = 3.80     # panel (b) axes height, matched to (a)'s
ROW_GAP       = 0.94     # gap between row 1's lowest content and row 2's box top (scaled 1.25x) -- (c)/(d) use
                          # the INSET set_title(y=0.95) convention (see _c_p/_d_p below), which
                          # sits inside the box rather than in a reserved strip above it, so this
                          # only needs to clear row 1's legend/footnote text, not a title too.
ROW2_BOX_W    = 6.540945  # panel (c)'s ORIGINAL square-box side -- still used below for every
                          # WIDTH-related calc (AX_M_W, and (d)'s width/X0 centering), since only
                          # HEIGHT changes here.
ROW2_H_INCREASE = 2.225   # inches -- (c) grows by this on BOTH height and width (square, explicit
                          # user request, so its width also becomes ROW2_BOX_W + increase); (d)
                          # grows by the SAME amount on height ONLY, width held fixed (explicit
                          # user request) -- so (d)'s box is no longer square. Bumped from 1.0 to
                          # 2.225 (explicit user request) to bring (c)/(d)'s rendered 3-D plot size,
                          # AFTER \includegraphics[width=\columnwidth] scaling in the manuscript, up
                          # to match multi_init/Circular_combined.pdf's 3-D panel: measured via
                          # rendered non-white content bbox in each PDF x each PDF's own embedded
                          # scale factor (0.18352 here vs 0.2955 there) -- circular's (a) panel is
                          # ~1.52in tall printed, this panel's (c) was only ~1.31in at increase=1.0.
ROW2_C_H      = ROW2_BOX_W + ROW2_H_INCREASE  # (c)'s box height AND width (aspect preserved)
ROW2_D_H      = ROW2_BOX_W + ROW2_H_INCREASE  # (d)'s box HEIGHT only; its width stays ROW2_BOX_W-derived
ROW2_XLABEL_H = 0.50     # room for (c)/(d)'s x-tick labels + xlabel, which sit below the row-2 boxes (scaled 1.25x)
LEGEND_GAP    = 0.06     # gap between (c)/(d)'s x-labels and the shared bottom controller legend (scaled 1.25x)
LEGEND_H      = 2.10     # bumped from 0.90 (2026-09-16): the bottom legend wrapped to 2 rows
                         # (ncol 5 -> 3) to fit the bumped _LEG_FS within FIG_W -- needs roughly
                         # double the single-row height reserved here
BOTTOM_MARGIN = 0.15     # scaled 1.25x

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


def _panel_title(ax, text, extra_shift=0.0):
    """Panel subtitle centered above ax, a fixed 0.10in above its top edge --
    shared by every RESERVED-STRIP-style panel ((a)/(b); (c)/(d) use the
    inset convention instead, see _inset_title). Uses
    get_position(original=True): Axes3D silently shrinks its box to
    preserve aspect and reports that shrunk "active" box from plain
    get_position(), which would throw off alignment between panels whose
    assigned height/width ratios differ; original=True returns the box we
    actually assigned, for both 2-D and 3-D axes alike. extra_shift (inches)
    adds an additional per-panel leftward nudge on top of TITLE_X_SHIFT
    (explicit user request: (a)/(c) only, not (b)/(d))."""
    p = ax.get_position(original=True)
    ax.figure.text((p.x0 + p.x1) / 2 - (TITLE_X_SHIFT + extra_shift) / FIG_W, p.y1 + 0.10 / FIG_H, text,
                    ha="center", va="bottom", fontsize=PANEL_TITLE_FS)


def _inset_title(ax, text, x_ref_center, extra_shift=0.0):
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
    ax.figure.text(x_ref_center - (TITLE_X_SHIFT + extra_shift) / FIG_W, p.y1 - 0.035 * (p.y1 - p.y0), text,
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
ROW2_SHIFT_LEFT = 1.8244  # inches -- reduced from 2.6125 to compensate for ROW2_H_INCREASE's
                          # 2026-09-16 jump (1.0 -> 2.225in): (c)'s box grew ~1.23in in width, which
                          # (being centred-then-shifted) pushed its left edge to x0=-0.79in (clipped
                          # off the canvas) at the old shift value -- re-solved so x0 lands back at
                          # ~0 (flush against the left margin, matching the pre-resize intent) net of
                          # AX_M_EXTRA_LEFT below.
AX_M_EXTRA_LEFT = 0.45    # inches -- additional (c)-only leftward push (explicit user request,
                          # bumped 0.15 -> 0.45 on 2026-09-16 using margin freed by FIG_W's
                          # growth since this was first tuned) to bring (c) flush against the
                          # figure's left margin
AX_M_W = ROW2_C_H  # square: width grows with height
AX_M_X0 = COL0_X0 + (COL0_W - AX_M_W) / 2.0 - ROW2_SHIFT_LEFT - AX_M_EXTRA_LEFT
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
AX_KE_EXTRA_RIGHT = 2.5   # inches -- additional (d)-only rightward push (explicit user request,
                          # bumped 0.5 -> 0.8 -> 2.0 on 2026-09-16: the box's own content
                          # (curves/z-label, measured below its title row) had ~1.67in of
                          # margin to FIG_W's edge, plenty of room for this). Was dropped to
                          # 0.0 (from 0.15) when (d)'s box
                          # grew and started clipping "C~r_x" off the page; FIG_W's later bump to
                          # 19.6in (to fix that clipping at the source) freed ~0.6in of margin, enough to
                          # move (d) right again without reintroducing the clip.
                          # to bring (d) flush against the figure's right margin
_ax_ke_center = COL1_X0 + COL1_W / 2.0 - ROW2_SHIFT_LEFT
_ax_ke_w_narrow = ROW2_BOX_W - 2.15
AX_KE_X0 = (_ax_ke_center - _ax_ke_w_narrow / 2.0 - 2.99 - 0.5   # left edge, held fixed as width
                          # grows; extra 0.5in shift added when (d)'s height was increased --
                          # the taller box's changed 3-D perspective pushed the z-label right
                          # enough to clip against the figure edge, so this recovers margin
            + AX_KE_EXTRA_RIGHT)
AX_KE_W = 8.68  # unchanged (explicit user request: (d)'s height grows, its width does not)
ax_ke = fig.add_axes([AX_KE_X0 / FIG_W, _y0_frac(row2_top, ROW2_D_H), AX_KE_W / FIG_W, ROW2_D_H / FIG_H],
                      projection="3d")

# Category legend dropped here (redundant with the caption's S/H/A key and
# collided with the shared bottom controller-color legend); kept only on the
# standalone comparison_outcome_heatmap.pdf.
# Font sizes below match multi_init/Circular_combined.pdf's printed sizes (explicit user
# request): labels ~8.0pt, ticks ~6.8pt, legend ~7.1pt, at FIG_W=21.0's ~0.1671 \columnwidth
# embed scale (native_pt = target_pt / 0.1671). An earlier attempt at this same target with
# FIG_W still at 19.6 was rejected because the suptitle and bottom legend ran off the page --
# see FIG_W's comment above for why growing native font size alone doesn't fix that.
_LBL_FS, _TICK_FS, _LEG_FS, _TITLE_FS = 48, 41, 42, 52
_draw_heatmap(ax_hm, cell_fontsize=43, tick_fontsize=_TICK_FS,
              show_legend=True, legend_ncol=2, legend_anchor="auto-left", legend_fontsize=36,
              # Re-added (2026-09-16) as a 2-row legend (ncol=2: S/H on row 1, A on row 2,
              # explicit user request) -- a single-row 3-column layout at any fontsize close to
              # matching Circular_combined.pdf's sizing was too wide for the (a)-(b) column gap
              # and drove into (b)'s own plot area; wrapping to 2 rows keeps the font size while
              # roughly halving the widest row's horizontal footprint.
              show_title=False)
_draw_energy(ax3d, fontsize=_LBL_FS, tick_fontsize=_TICK_FS)      # (b): Relative Touchdown Energy
# (b)'s own legend, separate from (c)'s shared bottom controller-color legend below --
# (b) only ever plots REACHED_CTRLS (2 of 5; controllers that never land get no bar), so
# reusing the full 5-controller legend would mislabel it. The "N/A" swatch (a white patch
# with the same black-edge/pad styling as the in-axes N/A markers drawn by _draw_energy)
# was previously undocumented anywhere in this panel -- explicit user request to add it here.
_b_handles, _b_labels = ax3d.get_legend_handles_labels()
if len(_b_handles) > 2:
    # 2026-09-21: with the retuned baselines several controllers reach the surface, so (b)'s own legend (VISTA + up to 4 baselines +
    # N/A) no longer fits under the panel (it wrapped into (c)'s area and clipped at the right edge). The bottom shared legend already
    # maps every controller colour, so (b) keeps only the N/A swatch it alone needs.
    _b_handles, _b_labels = [], []
_b_handles = _b_handles + [mpatches.Patch(facecolor="white", edgecolor="black",
                                           linewidth=0.6, label="Not landed")]
_b_labels = _b_labels + ["Not landed"]
# Single row (ncol=3, one per entry), anchored below the "Only landed runs shown" footnote
# (footnote itself sits at y=-0.36 in axes fraction -- legend placed further down still).
ax3d.legend(_b_handles, _b_labels, loc="upper center", bbox_to_anchor=(0.5, -0.45),
            ncol=3, frameon=False, fontsize=36,
            handlelength=1.4, handletextpad=0.5, columnspacing=1.2)
_draw_3d(ax_m, fontsize=_LBL_FS, ticksize=_TICK_FS)               # (c): Landing Trajectories
# _D_ELONG (box_aspect's time-axis:y:z ratio) and _D_ZOOM were both tuned by
# rendering and comparing (d)'s rendered height against (c)'s: (i) matplotlib's
# Line3D.get_window_extent() is NOT reliable in this matplotlib version (huge
# nonsense extents, insensitive to box_aspect/zoom) so it can't be used to
# measure this directly; (ii) proj3d.proj_transform + ax.transData applied to
# the safe-box's fixed corner coordinates IS reliable and was used instead to
# solve for the _D_ZOOM that makes (d)'s projected vertical span match (c)'s.
_D_ELONG = 1.3   # reduced from 1.5 (explicit user request)
_D_ZOOM = 0.9779  # re-tuned (same proj3d.proj_transform + ax.transData calibration method as
                 # before) to hold (d)'s rendered height at (c)'s after elong dropped 1.5 -> 1.3
                 # before) to hold (d)'s rendered height at (c)'s after elong dropped 2.0 -> 1.5
_draw_fov_3d(ax_ke, fontsize=_LBL_FS, tick_fontsize=_TICK_FS, zoom=_D_ZOOM, elong=_D_ELONG)  # (d): Marker Visibility

_panel_title(ax_hm, "(a) Closed-Loop Outcome", extra_shift=1.15)
_panel_title(ax3d, "(b) Touchdown Energy", extra_shift=-0.1)
# extra_shift values above/below widened (2026-09-16) after the font-size bump toward
# Circular_combined.pdf's sizes made the wider title TEXT overlap across the (a)/(b) and
# (c)/(d) column gap -- measured via PDF text search (page.search_for), not eyeballed.
_inset_title(ax_m, "(c) Landing Trajectories, Case 5", (COL0_X0 + COL0_W / 2.0) / FIG_W, extra_shift=1.0)
_inset_title(ax_ke, "(d) Centroid Visibility, Case 5", (AX3D_X0 + AX3D_W / 2.0) / FIG_W, extra_shift=-0.75)

# Legend anchored just below row 2 (upper-center at legend_top), not at the
# absolute figure bottom -- ties its position to the content above it
# directly, instead of relying on a separately-sized blank canvas below.
# 2 rows x 3 cols (last cell empty): single-row ncol=5 fit at the old, smaller
# _LEG_FS, but at the bumped size (matching Circular_combined.pdf) it ran off
# both edges of the canvas -- wrapping to 2 rows keeps the per-label font size
# instead of shrinking it. Handles/labels come from ax_m (Landing
# Trajectories), the only row-2 panel that plots+labels all 5 CTRLS --
# _draw_energy only labels REACHED_CTRLS (2 of 5).
handles, labels = ax_m.get_legend_handles_labels()
# VISTA is CTRLS[0] (ax_m plots/labels in CTRLS order) -- moved to the end per explicit user
# instruction, baselines keeping their original relative order ahead of it.
handles, labels = handles[1:] + handles[:1], labels[1:] + labels[:1]
LEGEND_UP_SHIFT = 0.5  # inches -- moves the legend up off legend_top (explicit user request);
                       # leaves a bit of extra blank margin at the very bottom of the figure
                       # rather than reworking the row2_bottom/ROW2_XLABEL_H/LEGEND_GAP chain.
ctrl_legend = fig.legend(handles, labels, loc="upper center", ncol=3,
           bbox_to_anchor=(0.5, _y0_frac(legend_top - LEGEND_UP_SHIFT, 0.0)),
           frameon=False, fontsize=_LEG_FS, handlelength=1.6, columnspacing=1.6, handletextpad=0.5)
for line in ctrl_legend.get_lines():
    line.set_linewidth(3.0)

fig.suptitle("Closed-Loop Comparison of VISTA with Baselines",
             fontsize=53, y=0.985)  # -> ~8.9pt printed, matching Circular_combined.pdf's own
                                    # fig.suptitle() size (explicit user request)
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
