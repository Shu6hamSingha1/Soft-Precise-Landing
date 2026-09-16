#!/usr/bin/env python3
"""
Render the 3 static end-of-landing plots used in tools/make_landing_montage.py's
animated SIDE panel (there they sweep a time cursor in sync with video; here they're
rendered full/final, as separate PDFs), with axis/legend labels using the manuscript's
notation (Soft_Precise_Landing/manuscript.tex):
  1. UAV & target 3D trajectory      -- {}^I p_b, {}^I p_t in the inertial frame I
     (NED; axes X_i, Y_i, Z_i per Sec. "PRELIMINARIES & PROBLEM FORMULATION")
  2. || {}^I r_{t/b} || (m) vs t      -- relative position (eq. around line 258:
     ^I r_{t/b,xy}(t_f) <= delta_r)
  3. || {}^I v_{t/b} || (m/s) vs t    -- relative velocity (^I v_{t/b} -> 0, delta_v)
Manuscript defines r_{t/b}/v_{t/b} (target relative to body/UAV) and the inertial
frame I, but not standalone absolute-position symbols; p_b/p_t here is a light,
consistent extension (b = body/UAV, t = target, matching the manuscript's own
subscript convention) for the per-vehicle 3D trace, not a manuscript equation.
Left superscripts (^I) use matplotlib mathtext's empty-base trick ({}^{...}); real
usetex isn't available in this environment (missing type1ec.sty), so bold vectors
use mathtext's \boldsymbol instead of full LaTeX usetex.

FONT NOTE (2026-09-16): matplotlib's built-in 'cm' mathtext fontset (BakomaFonts)
never bundled a bold-italic Computer Modern math font (cmmib10) -- its internal
_fontmap only has cal/rm/tt/it/bf/sf/ex, no 'bfit'. \boldsymbol on a Latin letter
is tagged font class "bfit", which BakomaFonts can't resolve, so it silently
falls back to STIXGeneral-BoldItalic instead of genuine CM. This diverges from
the manuscript's actual embedded font (confirmed via `ICRA_manuscript.pdf`'s font
list: real Type1 CMMIB10, from amsmath/bm).
FIX (2026-09-16): mathtext.fontset="custom" with every slot pinned explicitly,
using tools/fonts/lmmib10.ttf for the 'bfit' (bold-italic) slot -- a from-scratch
TrueType conversion (via tools/fonts/../../../scratchpad convert script, fontTools
t1Lib + Cu2QuPen) of the REAL lmmib10.pfb (Latin Modern Math Italic Bold, the
actively-maintained metrically-compatible clone of cmmib10 shipped by every TeX
distro), registered at runtime via fm.fontManager.addfont(). An earlier attempt
using tools/fonts/jsMath-cmmib10.ttf (an old hand-hacked jsMath-project TTF) also
rendered correctly but made every PDF fail to open in a standard viewer ("a number
is out of range") -- traced to matplotlib's PDF backend defaulting to
pdf.fonttype=3 (Type 3: each glyph re-drawn as tiny PDF vector-path subroutines
rather than embedding real outlines), a combination known to trip Acrobat's
strict validator. Fixed by (a) using a cleanly-built TTF instead of the jsMath
file, AND (b) forcing pdf.fonttype=42 (embeds actual TrueType outlines) below --
belt and suspenders, since either alone may have been sufficient but both are
cheap. If PDFs from this script ever fail to open again, suspect this pairing
first and re-verify in an actual PDF viewer (PyMuPDF is too lenient to catch it).

Usage:
  tools/plot_landing_summary.py --run "test_data/Landing_Test_Cross/Fri Aug 28 12-14-32 2026" \
      --out test_data/Test_Videos/summary_lissajous_cross
"""
import argparse
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

_LMMIB10_TTF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fonts", "lmmib10.ttf")
if os.path.exists(_LMMIB10_TTF):
    fm.fontManager.addfont(_LMMIB10_TTF)

# Match the manuscript's LaTeX-rendered labels (every MATLAB plotter uses
# Interpreter='latex', e.g. plotter_adaptive.m:85-87 "$^Ix$ (m)" etc. -- Computer
# Modern math, not matplotlib's DejaVu-based mathtext default). Real usetex=True
# fails in this environment (missing type1ec.sty / cm-super, no sudo), so use
# matplotlib's 'custom' mathtext fontset with every slot pinned to a genuine CM
# (or CM-compatible) font file (see FONT NOTE above) -- no external LaTeX
# dependency, matches MATLAB's Interpreter='latex' output AND the manuscript's
# real CMMIB10 for \boldsymbol.
# Font sizes aligned to the 9/10/8pt scheme used by scripts/make_multi_init_plots.py
# (and the other MATLAB-data plot scripts) so this figure matches Circular_combined.pdf.
plt.rcParams.update({
    "figure.dpi": 600,
    "savefig.dpi": 600,
    "pdf.fonttype": 42,
    "font.family": "serif",
    "font.serif": ["cmr10", "Computer Modern Roman", "DejaVu Serif"],
    "mathtext.fontset": "custom",
    "mathtext.rm": "cmr10",
    "mathtext.it": "cmmi10",
    "mathtext.bf": "cmb10",
    "mathtext.bfit": "LMMathBoldItalic10" if os.path.exists(_LMMIB10_TTF) else "cmmi10",
    "mathtext.cal": "cmsy10",
    "mathtext.sf": "cmss10",
    "mathtext.tt": "cmtt10",
    "axes.formatter.use_mathtext": True,
    "font.size": 27,
    "axes.labelsize": 27,
    "axes.titlesize": 30,
    "legend.fontsize": 24,
    "xtick.labelsize": 24,
    "ytick.labelsize": 24,
    "axes.grid": True,
    "grid.alpha": 0.3,
})

_INK = "#141414"
C_UAV, C_TARGET, C_RPOS, C_RVEL = "#2a78d6", "#eb6834", "#e34948", "#008300"
_P_B = r"${}^{\mathcal{I}}\boldsymbol{r}_\mathrm{b}$"
_P_T = r"${}^{\mathcal{I}}\boldsymbol{r}_\mathrm{t}$"
_R_TB = r"$\|\,{}^{\mathcal{I}}\boldsymbol{r}_{\mathrm{rel}}\|$ (m)"
_V_TB = r"$\|\,{}^{\mathcal{I}}\boldsymbol{v}_{\mathrm{rel}}\|$ (m/s)"
_T = r"$t$ (s)"


def load_series(run_dir):
    gt = np.load(os.path.join(run_dir, "Ground_Truth.npy"), allow_pickle=True).item()
    up, tp = gt["UAV Pose"], gt["Target Pose"]
    t = np.asarray(gt["Time"], float)
    n = min(len(up), len(tp), len(t))
    up, tp, t = up[:n], tp[:n], t[:n]
    ux = np.array([p.position.x for p in up]); uy = np.array([p.position.y for p in up]); uz = np.array([p.position.z for p in up])
    tx = np.array([p.position.x for p in tp]); ty = np.array([p.position.y for p in tp]); tz = np.array([p.position.z for p in tp])
    t = t - t[0]
    rel = np.stack([ux - tx, uy - ty, uz - tz], axis=1)
    rpos = np.linalg.norm(rel, axis=1)
    dt = np.gradient(t); dt[dt <= 0] = 1e-3
    rvel_vec = np.stack([np.gradient(rel[:, i]) / dt for i in range(3)], axis=1)
    rvel = np.linalg.norm(rvel_vec, axis=1)
    rvel = np.convolve(rvel, np.ones(7) / 7, mode="same")
    # trim to touchdown = first min-altitude sample (same convention as the montage tool)
    td = int(np.argmin(uz - tz)) + 1
    sl = slice(0, td)
    return dict(t=t[sl], ux=ux[sl], uy=uy[sl], uz=uz[sl], tx=tx[sl], ty=ty[sl], tz=tz[sl],
                rpos=rpos[sl], rvel=rvel[sl])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--out", required=True, help="output path prefix (no extension)")
    a = ap.parse_args()
    s = load_series(a.run)

    fig = plt.figure(figsize=(6, 5))
    ax3 = fig.add_subplot(111, projection="3d")
    ax3.plot(s["ux"], s["uy"], s["uz"], color=C_UAV, lw=2.0, label=_P_B)
    ax3.plot(s["tx"], s["ty"], s["tz"], color=C_TARGET, lw=2.0, label=_P_T)
    ax3.scatter(s["ux"][-1], s["uy"][-1], s["uz"][-1], color=C_UAV, s=40, marker="x")
    ax3.scatter(s["tx"][-1], s["ty"][-1], s["tz"][-1], color=C_TARGET, s=40, marker="x")
    ax3.set_xlabel(r"$\,^\mathcal{I}x$ [m]", labelpad=10, color=_INK)
    ax3.set_ylabel(r"$\,^\mathcal{I}y$ [m]", labelpad=10, color=_INK)
    ax3.set_zlabel(r"$\,^\mathcal{I}z$ [m]", labelpad=18, color=_INK)
    ax3.legend(loc="upper right", bbox_to_anchor=(1.18, 1.22))
    fig.tight_layout()
    fig.savefig(a.out + "_3d.pdf")
    plt.close(fig)

    for name, label, y, col, fh in [("rpos", _R_TB, s["rpos"], C_RPOS, 3.0),
                                     ("rvel", _V_TB, s["rvel"], C_RVEL, 3.9)]:
        fig, ax = plt.subplots(figsize=(6, fh))
        ax.plot(s["t"], y, color=col, lw=2.0)
        ax.set_xlabel(_T, color=_INK); ax.set_ylabel(label, color=_INK)
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(f"{a.out}_{name}.pdf")
        plt.close(fig)

    print(f"wrote {a.out}_3d.pdf, {a.out}_rpos.pdf, {a.out}_rvel.pdf")


if __name__ == "__main__":
    main()
