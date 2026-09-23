#!/usr/bin/env python3
"""Compose a 2x2 baseline-comparison montage per target profile from the four
comparison-controller recordings in test_data/Final/:

  top-left     PBVS-PPC   (LIN2022-GT)
  top-right    PBVS-AEDO  (ZHANG2026-GT)
  bottom-left  IBVS-PPC   (LIN2023-GT)
  bottom-right FF-IBVS    (CHO2022-GT)

Each panel is built fresh from that rep's *_chase_cam.mp4 + an animated GT-plot
overlay (reusing load_series/render_plots from make_landing_montage.py) --
NOT the precomposited *_montage.mp4, which also bakes in an onboard-cam PiP.
2026-09-23, user request: drop the onboard view, chase-cam + plots only.

Each panel is labeled with its abbreviation and a landing-quality tag drawn
from that rep's Ground_Truth.npy['SoftPrecise'] dict (same taxonomy as
tools/build_test_record.py::classify, collapsed to the 5 tags the user asked
for): soft-precise / soft-imprecise / hard-precise / hard-imprecise / aborted
(an empty SoftPrecise dict means the rep never reached touchdown detection --
still airborne/hovering at the end of the recording, i.e. aborted).

Title banner text is rendered in Times New Roman (2026-09-23, user request;
cv2.putText only offers Hershey fonts, so titles are drawn with Pillow using
the system msttcorefonts Times New Roman TTF, then composited back into the
OpenCV BGR frame).

Shorter reps freeze on their last frame once they end, so all four panels
run for the duration of the longest controller's rep.

Usage:
  tools/make_baseline_comparison_montage.py --profile Circular
  tools/make_baseline_comparison_montage.py --all
"""
import argparse
import os
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_landing_montage import load_series, render_plots  # noqa: E402

ROOT = os.path.expanduser("~/Soft-Precise-Landing/PX4_Gazebo")
FINAL = os.path.join(ROOT, "test_data", "Final")
OUT_DIR = os.path.join(FINAL, "Baseline_Comparison")

TIMES_NEW_ROMAN = "/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf"

# (controller dir, panel abbreviation) in the requested 2x2 sequence
PANELS = [
    ("LIN2022-GT", "PBVS-PPC"),
    ("ZHANG2026-GT", "PBVS-AEDO"),
    ("LIN2023-GT", "IBVS-PPC"),
    ("CHO2022-GT", "FF-IBVS"),
]
PROFILES = ["Static", "Circular", "Linear", "Lissajous", "Sinusoidal"]

# All tags render in yellow now (2026-09-23, user request); kept as a dict
# in case per-tag color is wanted again later.
TAG_COLOR = {
    "soft-precise": (0, 255, 255), "hard-precise": (0, 255, 255),
    "soft-imprecise": (0, 255, 255), "hard-imprecise": (0, 255, 255),
    "aborted": (0, 255, 255),
}


def tag_for(controller, profile):
    gt_path = os.path.join(FINAL, controller, profile, "dataset", "Ground_Truth.npy")
    sp = {}
    try:
        gt = np.load(gt_path, allow_pickle=True).item()
        sp = gt.get("SoftPrecise", {}) or {}
    except Exception:
        pass
    if not sp:
        return "aborted"           # never reached touchdown detection
    if bool(sp.get("target_lost", False)):
        return "aborted"
    prec = bool(sp.get("precise", False))
    soft = bool(sp.get("soft", False))
    if prec and soft:
        return "soft-precise"
    if prec and not soft:
        return "hard-precise"
    if soft and not prec:
        return "soft-imprecise"
    return "hard-imprecise"


def draw_label(tile, text, tag):
    """Solid black banner across the top of the tile: Times New Roman
    controller name (white) and landing-status tag (yellow) side by side on
    one line, both enlarged (2026-09-23, user request). The tag is set in
    small caps -- rendered upper-case at a slightly reduced size, the usual
    approximation of small caps for a font with no dedicated small-caps
    variant -- so it reads as a status marker distinct from the title."""
    h, w = tile.shape[:2]
    bar_h = max(52, int(h * 0.13))
    cv2.rectangle(tile, (0, 0), (w, bar_h), (0, 0, 0), -1)

    pil_img = Image.fromarray(cv2.cvtColor(tile, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_img)
    title_font = ImageFont.truetype(TIMES_NEW_ROMAN, int(bar_h * 0.62))
    tag_font = ImageFont.truetype(TIMES_NEW_ROMAN, int(bar_h * 0.48))

    title_y = (bar_h - title_font.size) // 2
    draw.text((14, title_y), text, font=title_font, fill=(255, 255, 255))
    title_w = draw.textlength(text, font=title_font)

    tag_bgr = TAG_COLOR.get(tag, (255, 255, 255))
    tag_y = (bar_h - tag_font.size) // 2 + int(title_font.size * 0.06)
    draw.text((14 + title_w + 24, tag_y), tag.upper(), font=tag_font,
              fill=(tag_bgr[2], tag_bgr[1], tag_bgr[0]))
    return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)


class Panel:
    """Chase-cam + animated GT-plot overlay for one controller/profile rep,
    sim-time synced the same way make_landing_montage.py syncs its chase
    panel (chase spans exactly [descent-start, touchdown])."""

    def __init__(self, controller, profile, abbrev, fps, tail_s=1.0):
        run_dir = os.path.join(FINAL, controller, profile, "dataset")
        chase_path = os.path.join(FINAL, controller, profile, f"{profile}_chase_cam.mp4")
        self.abbrev = abbrev
        self.tag = tag_for(controller, profile)
        self.chase = cv2.VideoCapture(chase_path)
        self.cN = int(self.chase.get(cv2.CAP_PROP_FRAME_COUNT))
        self.series = load_series(run_dir)
        self.gN = len(self.series["t"])
        self.dur = float(self.series["t"][-1])
        self.fps = fps
        self.cfps_eff = max(self.cN - 1, 1) / max(self.dur, 1e-6)
        self.tG_rel = np.asarray(self.series["t"], float)
        self.nd = max(1, int(self.dur * fps))
        self.nt = int(tail_s * fps)
        self.nframes = self.nd + self.nt
        pad = 0.5
        xs = np.concatenate([self.series["ux"], self.series["tx"]])
        ys = np.concatenate([self.series["uy"], self.series["ty"]])
        zs = np.concatenate([self.series["uz"], self.series["tz"]])
        self.lims = {"x": (xs.min() - pad, xs.max() + pad), "y": (ys.min() - pad, ys.max() + pad),
                     "z": (min(zs.min(), 0.0), zs.max() + pad)}
        self._ccache = {"i": -1, "f": None}
        self._last_tile = None

    def _grab_chase(self, idx):
        idx = max(0, idx)
        cache = self._ccache
        if cache["i"] == idx and cache["f"] is not None:
            return cache["f"]
        if idx < cache["i"]:
            self.chase.set(cv2.CAP_PROP_POS_FRAMES, idx)
            cache["i"] = idx - 1
        f = cache["f"]
        while cache["i"] < idx:
            ok, fr = self.chase.read()
            if not ok:
                break
            f = fr
            cache["i"] += 1
        cache["f"] = f
        return f

    def get_tile(self, global_f, tile_w, tile_h):
        f = min(global_f, self.nframes - 1)
        if f < self.nd:
            t_out = f / self.fps
            ci = int(np.clip(round(t_out * self.cfps_eff), 0, self.cN - 1))
            gi = int(np.clip(np.searchsorted(self.tG_rel, t_out), 0, self.gN - 1))
        else:
            ci = self.cN - 1
            gi = self.gN - 1
        cf = self._grab_chase(ci)
        if cf is None:
            cf = np.zeros((tile_h, tile_w, 3), np.uint8)
        tile = cv2.resize(cf, (tile_w, tile_h))

        plot_ph = int(tile_h * 0.92)
        plot_pw = int(plot_ph * 0.48)
        plot_rgba = render_plots(self.series, gi, plot_pw, plot_ph, self.lims)
        pm = 10
        py0, px0 = tile_h - plot_ph - pm, tile_w - plot_pw - pm
        alpha_ch = (plot_rgba[:, :, 3:4].astype(np.float32) / 255.0) * 0.75
        plot_bgr = cv2.cvtColor(plot_rgba[:, :, :3], cv2.COLOR_RGB2BGR).astype(np.float32)
        roi = tile[py0:py0 + plot_ph, px0:px0 + plot_pw].astype(np.float32)
        blended = alpha_ch * plot_bgr + (1.0 - alpha_ch) * roi
        tile[py0:py0 + plot_ph, px0:px0 + plot_pw] = blended.astype(np.uint8)

        tile = draw_label(tile, self.abbrev, self.tag)
        self._last_tile = tile
        return tile

    def release(self):
        self.chase.release()


def build_profile(profile, tile_w=640, tile_h=480, fps=25.0):
    panels = [Panel(controller, profile, abbrev, fps) for controller, abbrev in PANELS]
    nframes = max(p.nframes for p in panels)
    W, H = tile_w * 2, tile_h * 2
    os.makedirs(OUT_DIR, exist_ok=True)
    out_path = os.path.join(OUT_DIR, f"{profile}_2x2.mp4")
    vw = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))
    print(f"[2x2] {profile}: {nframes} frames @ {fps}fps -> {out_path}", flush=True)
    for f in range(nframes):
        tiles = [p.get_tile(f, tile_w, tile_h) for p in panels]
        top = np.hstack([tiles[0], tiles[1]])
        bottom = np.hstack([tiles[2], tiles[3]])
        canvas = np.vstack([top, bottom])
        vw.write(canvas)
        if f % 50 == 0:
            print(f"[2x2]  {profile} frame {f}/{nframes}", flush=True)
    vw.release()
    for p in panels:
        p.release()
    print(f"[2x2] done -> {out_path}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", choices=PROFILES)
    ap.add_argument("--all", action="store_true")
    a = ap.parse_args()
    profiles = PROFILES if a.all else [a.profile]
    if not profiles or profiles == [None]:
        ap.error("pass --profile <name> or --all")
    for p in profiles:
        build_profile(p)


if __name__ == "__main__":
    main()
