"""
make_comparison_qualplot.py

Rebuilds Figures/generated/comparison_combined_circular.pdf and adds a new
Figures/generated/comparison_outcome_heatmap.pdf.

WHY: the old comparison_combined_circular.pdf (see make_comparison_plots.py,
"Plot H") was a 1x4 of [3D Circular] + bars of {landing time, final xy error,
touchdown speed}. Once >=3 of the 4 baselines abort on EVERY case (never
reach the target surface), those three bars are measured at the moment of
abort, not at a landing -- "final xy error" and "touchdown speed" for an
airborne abort are not landing-precision/softness numbers at all, and
"landing time" is actively misleading (an early abort reads as "fast").

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
CTRL_DISPLAY = {
    "PLASMC (Proposed)": "VISTA (Proposed)",
    "Lin 2022":          "Baseline A (PBVS--PPC)",
    "Zhang 2026":        "Baseline B (PBVS--AEDO)",
    "Lin 2023":          "Baseline C (IBVS--PPC)",
    "Cho 2022":          "Baseline D (FF--IBVS)",
}
ROW_LABELS = {
    "PLASMC (Proposed)": "VISTA",
    "Lin 2022":          "Lin 2022",
    "Zhang 2026":        "Zhang 2026",
    "Lin 2023":          "Lin 2023",
    "Cho 2022":          "Cho 2022",
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


def _draw_heatmap(ax, title_fontsize=14, cell_fontsize=13, tick_fontsize=11, legend_fontsize=10):
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
    ax.set_title("Closed-Loop Outcome by Controller and Case", fontsize=title_fontsize, pad=10)
    legend_handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in CAT_COLORS]
    ax.legend(legend_handles, ["S soft-precise touchdown", "H hard/imprecise touchdown", "A aborted (did not reach surface)"],
              loc="upper left", bbox_to_anchor=(0.0, -0.13), ncol=1, frameon=False, fontsize=legend_fontsize)


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


def _draw_energy(ax, fontsize=16, title_fontsize=17, tick_fontsize=14):
    """Relative touchdown energy eta_E = (v_f/v_soft)^2, controllers that
    ever reach the surface only (VISTA, Zhang 2026) -- no bar slots wasted
    on controllers that never land."""
    xb = np.arange(len(TRAJS))
    width_b = 0.30
    xj_off = np.linspace(-0.5, 0.5, len(REACHED_CTRLS) + 2)[1:-1] * width_b * 2
    for j, name in enumerate(REACHED_CTRLS):
        eta = np.array([(METRICS[(tr, name)]["v_term"] / SOFT_V_REL_MPS) ** 2
                        if METRICS[(tr, name)]["reached"] else np.nan for tr in TRAJS])
        mask = ~np.isnan(eta)
        xj = xb + xj_off[j]
        ax.bar(xj[mask], eta[mask], width_b, color=CTRL_COLORS[name], label=CTRL_DISPLAY[name])
        for xk in xj[~mask]:
            ax.text(xk, 0.05, "N/A", rotation=90, ha="center", va="bottom",
                    fontsize=9, color=CTRL_COLORS[name])
    ax.axhline(1.0, color="k", lw=0.8, ls=":")
    ax.text(xb[-1] + 0.55, 1.0, r"$\eta_E=1$", fontsize=11, va="bottom", ha="right")
    ax.set_xticks(xb); ax.set_xticklabels(LABELS, rotation=20, fontsize=tick_fontsize)
    ax.tick_params(axis="y", labelsize=tick_fontsize)
    ax.grid(axis="y", alpha=0.3)
    ax.set_ylim(bottom=0)
    ax.set_ylabel(r"$E_{\mathrm{rel},f}/E_\mathrm{soft}$", fontsize=fontsize, labelpad=4)


def _draw_fov_margin(ax, fontsize=16, tick_fontsize=14):
    """FoV-margin time series, Case 5, all 5 controllers -- the
    mechanism-explaining panel: why each baseline fails."""
    for name in CTRLS:
        run = RUNS[(CASE5, name)]
        N = METRICS[(CASE5, name)]["N"]
        margin = _fov_margin(run, N)            # may trim further than N -- see docstring
        t = run.data.tRange[:len(margin)]
        ax.plot(t, margin, color=CTRL_COLORS[name], lw=1.4, label=CTRL_DISPLAY[name])
    ax.axhline(0.0, color="k", lw=0.8, ls=":")
    ax.text(0.3, 0.02, "FoV edge", fontsize=11, va="bottom")
    ax.set_ylim(bottom=min(-0.05, ax.get_ylim()[0]))
    ax.set_xlabel("$t$ [s]", fontsize=fontsize, labelpad=4)
    ax.set_ylabel("normalized FoV margin", fontsize=fontsize, labelpad=4)
    ax.tick_params(labelsize=tick_fontsize)
    ax.grid(alpha=0.3)


# ============================================================================
# Figure 1: comparison_combined_circular.pdf (2x2) -- the main comparison
#   figure, per the recommended set (2026-09-11): (a) outcome heatmap --
#   overall comparison, which controller lands/aborts/is hard/soft; (b) the
#   Circular (Case 5) 3D trajectory -- a representative qualitative case
#   study, the richest/most demanding case; (c) FoV margin, Case 5 -- WHY
#   baselines fail, not just that they do; (d) relative touchdown energy --
#   soft-landing quality as a severity measure, not a binary pass/fail.
# ============================================================================
fig = plt.figure(figsize=(11.0, 8.6))
ax_hm = fig.add_subplot(2, 2, 1)
ax3d  = fig.add_subplot(2, 2, 2, projection="3d")
ax_m  = fig.add_subplot(2, 2, 3)
ax_ke = fig.add_subplot(2, 2, 4)

_draw_heatmap(ax_hm, title_fontsize=15, cell_fontsize=14, tick_fontsize=12, legend_fontsize=10)
ax_hm.set_title("(a) Closed-Loop Outcome by Controller and Case", fontsize=15, pad=10)

_draw_3d(ax3d)
ax3d.set_title("(b) Landing Trajectories, Case 5", fontsize=17, y=1.0)

_draw_fov_margin(ax_m)
ax_m.set_title("(c) FoV Margin, Case 5", fontsize=17, y=1.03)

_draw_energy(ax_ke)
ax_ke.set_title("(d) Relative Touchdown Energy", fontsize=17, y=1.03)

handles, labels = ax3d.get_legend_handles_labels()
fig.legend(handles, labels, loc="lower center", ncol=3, bbox_to_anchor=(0.5, 0.0),
           frameon=False, fontsize=13, handlelength=1.6, columnspacing=2.0, handletextpad=0.6)
fig.tight_layout(rect=(0, 0.06, 1, 1), h_pad=4.0, w_pad=3.0)

safe_savefig(fig, f"{OUT}/comparison_combined_circular.pdf", pad_inches=0.05)
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
