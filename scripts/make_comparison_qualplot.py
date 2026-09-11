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
-- soft-precise / hard-imprecise / visibility-loss -- text-annotated.

Classification (thresholds match the rest of the manuscript):
  reached the surface <=> altitude above target at termination <= Z_REACH_M
  soft-precise         <=> reached AND r_xy <= PRECISE_XY_M AND v_term <= SOFT_V_REL_MPS
  hard/imprecise        <=> reached AND NOT soft-precise
  visibility-loss       <=> NOT reached (broke feature visibility, or timed out)
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
    "PLASMC (Proposed)": "VDF-ASMC (Proposed)",
    "Lin 2022":          "Baseline A [1] (PBVS--PPC)",
    "Zhang 2026":        "Baseline B [2] (PBVS--AEDO)",
    "Lin 2023":          "Baseline C [10] (IBVS--PPC)",
    "Cho 2022":          "Baseline D [9] (FF--IBVS)",
}
ROW_LABELS = {
    "PLASMC (Proposed)": "Proposed",
    "Lin 2022":          "Lin 2022 [1]",
    "Zhang 2026":        "Zhang 2026 [2]",
    "Lin 2023":          "Lin 2023 [10]",
    "Cho 2022":          "Cho 2022 [9]",
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
        cat = "loss"
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
    save it; re-run the comparison if this raises)."""
    d = run.data
    P = d.P_DS[:, :, :N]
    Np = P.shape[1] // 3
    cnp = P[:, 2 * Np:3 * Np, :]                     # physical camera corners [px]
    mx = (RES[0] / 2 - np.abs(cnp[0])) / (RES[0] / 2)
    my = (RES[1] / 2 - np.abs(cnp[1])) / (RES[1] / 2)
    return np.minimum(mx, my).min(axis=0)            # (N,)


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

# ============================================================================
# Figure 1: comparison_combined_circular.pdf (1x4)
# ============================================================================
fig = plt.figure(figsize=(18.0, 5.0))
gs  = fig.add_gridspec(1, 4, left=0.05, right=0.99, wspace=0.40)
ax3d  = fig.add_subplot(gs[0, 0], projection="3d")
ax_h  = fig.add_subplot(gs[0, 1])
ax_ke = fig.add_subplot(gs[0, 2])
ax_m  = fig.add_subplot(gs[0, 3])

# --- Panel 1: 3D Circular (Case 5) trajectories, corrected outcome markers ---
CASE5 = "Circular"
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
    marker = {"soft-precise": "^", "hard-imprecise": "o", "loss": "x"}[met["cat"]]
    if met["cat"] == "hard-imprecise":
        ax3d.scatter(X[0, -1], X[1, -1], -X[2, -1], facecolors="none", edgecolors=color,
                     marker=marker, s=34, linewidths=1.2)
    else:
        ax3d.scatter(X[0, -1], X[1, -1], -X[2, -1], color=color, marker=marker, s=34)
    if not target_drawn:
        xt = d.x_t[:3, :N]
        draw_landing_corridor(ax3d, xt[0], xt[1], xt[2])
        target_drawn = True
ax3d.set_xlabel(r"$\,^\mathcal{I}x$ [m]", labelpad=12, fontsize=18)
ax3d.set_ylabel(r"$\,^\mathcal{I}y$ [m]", labelpad=12, fontsize=18)
ax3d.set_zlabel("altitude [m]", labelpad=2, fontsize=18)
ax3d.locator_params(axis="x", nbins=4)
ax3d.locator_params(axis="y", nbins=4)
ax3d.locator_params(axis="z", nbins=4)
ax3d.tick_params(pad=1, labelsize=13)
ax3d.set_title("Landing Trajectories, Case 5", fontsize=18, x=0.55, y=0.95)
ax3d.view_init(elev=22, azim=-58)

# --- Panel 2: altitude above target at termination, all 5 cases ---
xb = np.arange(len(TRAJS))
width = 0.16
for j, name in enumerate(CTRLS):
    vals = [METRICS[(tr, name)]["h_term"] for tr in TRAJS]
    ax_h.bar(xb + (j - 2) * width, vals, width, color=CTRL_COLORS[name])
ax_h.axhline(Z_REACH_M, color="k", lw=0.8, ls=":")
ax_h.text(xb[-1] + 0.55, Z_REACH_M, "reached", fontsize=11, va="bottom", ha="right")
ax_h.set_xticks(xb); ax_h.set_xticklabels(LABELS, rotation=20, fontsize=14)
ax_h.tick_params(axis="y", labelsize=14)
ax_h.grid(axis="y", alpha=0.3)
ax_h.set_ylabel("altitude above target\nat termination [m]", fontsize=16, labelpad=4)
ax_h.set_title("Descent Reached", fontsize=18, y=1.03)

# --- Panel 3: terminal kinetic energy where reached (gap where aborted) ---
for j, name in enumerate(CTRLS):
    vals = np.array([METRICS[(tr, name)]["ke"] for tr in TRAJS])
    mask = ~np.isnan(vals)
    ax_ke.bar((xb + (j - 2) * width)[mask], vals[mask], width, color=CTRL_COLORS[name])
ax_ke.set_xticks(xb); ax_ke.set_xticklabels(LABELS, rotation=20, fontsize=14)
ax_ke.tick_params(axis="y", labelsize=14)
ax_ke.grid(axis="y", alpha=0.3)
ax_ke.set_ylabel(r"terminal K.E. $\frac{1}{2} m\|v_\mathrm{rel}\|^2$ [J]", fontsize=16, labelpad=4)
ax_ke.set_title("Touchdown Severity\n(bar absent = did not reach)", fontsize=16, y=1.0)

# --- Panel 4: FoV-margin time series, Case 5, all 5 controllers ---
for name in CTRLS:
    run = RUNS[(CASE5, name)]
    N = METRICS[(CASE5, name)]["N"]
    t = run.data.tRange[:N]
    margin = _fov_margin(run, N)
    ax_m.plot(t, margin, color=CTRL_COLORS[name], lw=1.4)
ax_m.axhline(0.0, color="k", lw=0.8, ls=":")
ax_m.text(0.3, 0.02, "FoV edge", fontsize=11, va="bottom")
ax_m.set_ylim(bottom=min(-0.05, ax_m.get_ylim()[0]))
ax_m.set_xlabel("$t$ [s]", fontsize=16, labelpad=4)
ax_m.set_ylabel("FoV margin [frac. of half-frame]", fontsize=16, labelpad=4)
ax_m.set_title("Visibility Margin, Case 5", fontsize=18, y=1.03)
ax_m.tick_params(labelsize=14)
ax_m.grid(alpha=0.3)

handles, labels = ax3d.get_legend_handles_labels()
fig.legend(handles, labels, loc="lower center", ncol=5, bbox_to_anchor=(0.5, 0.0),
           frameon=False, fontsize=13, handlelength=1.6, columnspacing=2.0, handletextpad=0.6)
fig.suptitle("Closed-Loop Comparison of Five Controllers across Cases 1--5",
             fontsize=22, y=0.99)
fig.subplots_adjust(bottom=0.230, top=0.865)
ax3d.set_position([0.000, 0.10, 0.24, 0.84])
bar_y0, bar_h = 0.23, 0.56
bar_left, bar_right = 0.33, 0.99
gap = 0.035
n_bars = 3
bar_w = (bar_right - bar_left - (n_bars - 1) * gap) / n_bars
for k, ax in enumerate((ax_h, ax_ke, ax_m)):
    x0 = bar_left + k * (bar_w + gap)
    ax.set_position([x0, bar_y0, bar_w, bar_h])

_target1 = f"{OUT}/comparison_combined_circular.pdf"
_tmp1 = f"{OUT}/.tmp_comparison_combined_circular.pdf"
fig.savefig(_tmp1, pad_inches=0.05)
plt.close(fig)
try:
    os.replace(_tmp1, _target1)
    print(f"Wrote {_target1}")
except PermissionError:
    print(f"WARNING: {_target1} is locked (likely open in a viewer) -- wrote {_tmp1} instead. "
          f"Close the PDF and re-run, or manually replace it.")

# ============================================================================
# Figure 2: comparison_outcome_heatmap.pdf (NEW)
# ============================================================================
CAT_CODE = {"soft-precise": 0, "hard-imprecise": 1, "loss": 2}
CAT_TEXT = {"soft-precise": "soft-\nprecise", "hard-imprecise": "hard/\nimprecise", "loss": "visibility\nloss"}
CAT_COLORS = ["#2e7d32", "#f9a825", "#c62828"]   # green / amber / red

grid = np.array([[CAT_CODE[METRICS[(tr, name)]["cat"]] for tr in TRAJS] for name in CTRLS])

fig, ax = plt.subplots(figsize=(7.2, 3.6))
cmap = ListedColormap(CAT_COLORS)
norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N)
ax.imshow(grid, cmap=cmap, norm=norm, aspect="auto")
for i, name in enumerate(CTRLS):
    for j, tr in enumerate(TRAJS):
        met = METRICS[(tr, name)]
        cat = met["cat"]
        ax.text(j, i, CAT_TEXT[cat], ha="center", va="center", fontsize=9,
                color="white" if cat != "hard-imprecise" else "black", linespacing=1.1)
ax.set_xticks(range(len(TRAJS))); ax.set_xticklabels(LABELS, fontsize=11)
ax.set_yticks(range(len(CTRLS))); ax.set_yticklabels([ROW_LABELS[n] for n in CTRLS], fontsize=11)
ax.set_xticks(np.arange(-0.5, len(TRAJS), 1), minor=True)
ax.set_yticks(np.arange(-0.5, len(CTRLS), 1), minor=True)
ax.grid(which="minor", color="white", linewidth=2)
ax.tick_params(which="minor", length=0)
ax.tick_params(which="major", length=0)
for spine in ax.spines.values():
    spine.set_visible(False)
ax.set_title("Closed-Loop Outcome by Controller and Case", fontsize=14, pad=10)

legend_handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in CAT_COLORS]
ax.legend(legend_handles, ["soft-precise touchdown", "hard/imprecise touchdown", "visibility loss"],
          loc="upper center", bbox_to_anchor=(0.5, -0.18), ncol=3, frameon=False, fontsize=10)

fig.tight_layout()
fig.savefig(f"{OUT}/comparison_outcome_heatmap.pdf", bbox_inches="tight", pad_inches=0.05)
plt.close(fig)
print(f"Wrote {OUT}/comparison_outcome_heatmap.pdf")

# ============================================================================
# Console summary (for the manuscript prose)
# ============================================================================
print("\n%-16s" % "controller", *["%-10s" % l for l in LABELS])
for name in CTRLS:
    row = [METRICS[(tr, name)]["cat"] for tr in TRAJS]
    print("%-16s" % ROW_LABELS[name], *["%-10s" % c for c in row])
