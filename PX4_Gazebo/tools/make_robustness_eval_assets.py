#!/usr/bin/env python3
"""Build the cross-marker ROBUSTNESS eval assets: lighting / polarity / colour /
texture variants of the clean `cross_marker` world.

WHY. The detector eval set (test_data/DetectorFrameset) is all ONE lighting condition,
ONE polarity (dark cross on light plate) and ONE texture, so robustness to the user's
stated bar -- "different lighting conditions, different colour combinations of the marker
and textured background, robust to other perception noise" -- is currently UNMEASURABLE.
Tuning against it re-fits to one scene. See
Memory/px4/feedback_cross_detector_robustness_requirement.md.

Writes marker textures + model dirs + worlds OUTSIDE the repo (PX4 gz asset trees), each
with a dated .bak of anything it overwrites. Nothing here touches the live default world.

  python3 tools/make_robustness_eval_assets.py [--dry-run]
"""
import argparse, os, shutil, datetime, re
import numpy as np, cv2

GZ = os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz")
MODELS, WORLDS = os.path.join(GZ, "models"), os.path.join(GZ, "worlds")
SRC_MODEL, SRC_WORLD = os.path.join(MODELS, "cross_marker"), os.path.join(WORLDS, "cross_marker.sdf")
STAMP = datetime.datetime.now().strftime("%Y%m%d")

def backup(p):
    if os.path.exists(p):
        b = f"{p}.bak_before_robustness_{STAMP}"
        if not os.path.exists(b): shutil.copy2(p, b)

def make_marker_png(src, dst, mode):
    """mode: 'inv' = polarity flip (light cross on dark plate);
             'col' = chromatic, NEAR-ISO-LUMINANT (red cross on green plate) -- the case
                     an absolute-brightness gate cannot see at all."""
    im = cv2.imread(src, cv2.IMREAD_COLOR)
    g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    stroke = g < 100                      # the cross strokes (5.2% of pixels)
    out = np.zeros_like(im)
    if mode == "inv":
        out[...] = (25, 25, 25)           # dark plate
        out[stroke] = (235, 235, 235)     # light cross
    elif mode == "col":
        # BGR. Green plate / red cross, luminance matched to ~within 8 grey levels so the
        # ONLY separating signal is hue -- an absolute V gate is blind here by construction.
        out[...] = (40, 150, 40)
        out[stroke] = (40, 40, 150)
    else:
        raise ValueError(mode)
    backup(dst); cv2.imwrite(dst, out)
    lum = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
    return dict(plate=float(lum[~stroke].mean()), cross=float(lum[stroke].mean()))

def make_model(name, png_basename):
    d = os.path.join(MODELS, name); os.makedirs(d, exist_ok=True)
    sdf = open(os.path.join(SRC_MODEL, "model.sdf")).read()
    sdf = sdf.replace("model://cross_marker/cross_marker.png", f"model://{name}/{png_basename}")
    sdf = sdf.replace("<model name='cross_marker'>", f"<model name='{name}'>")
    sdf = sdf.replace('<model name="cross_marker">', f'<model name="{name}">')
    p = os.path.join(d, "model.sdf"); backup(p); open(p, "w").write(sdf)
    cfg = os.path.join(SRC_MODEL, "model.config")
    if os.path.exists(cfg):
        t = open(cfg).read().replace("cross_marker", name)
        open(os.path.join(d, "model.config"), "w").write(t)
    return d

def make_world(name, *, light_intensity=None, light_dir=None, marker_model=None, ground_rgb=None):
    s = open(SRC_WORLD).read()
    s = s.replace('<world name="cross_marker">', f'<world name="{name}">')
    if light_intensity is not None:
        s = re.sub(r"<intensity>[\d.]+</intensity>", f"<intensity>{light_intensity}</intensity>", s, count=1)
    if light_dir is not None:
        s = re.sub(r"<direction>[-\d.\s]+</direction>", f"<direction>{light_dir}</direction>", s, count=1)
    if marker_model is not None:
        s = s.replace("<uri>model://cross_marker</uri>", f"<uri>model://{marker_model}</uri>")
    if ground_rgb is not None:
        # first material block belongs to the ground plane visual
        s = s.replace("<ambient>0.8 0.8 0.8 1</ambient>", f"<ambient>{ground_rgb}</ambient>", 1)
        s = s.replace("<diffuse>0.8 0.8 0.8 1</diffuse>", f"<diffuse>{ground_rgb}</diffuse>", 1)
    p = os.path.join(WORLDS, f"{name}.sdf"); backup(p); open(p, "w").write(s)
    return p

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--dry-run", action="store_true"); a = ap.parse_args()
    if a.dry_run:
        print("would write markers cross_marker_inv/col, and worlds: dim, bright, lowsun, inv, col, darkbg"); return
    r1 = make_marker_png(os.path.join(SRC_MODEL, "cross_marker.png"),
                         os.path.join(MODELS, "cross_marker_inv", "cross_marker_inv.png") if os.makedirs(os.path.join(MODELS,"cross_marker_inv"), exist_ok=True) is None else "", "inv")
    make_model("cross_marker_inv", "cross_marker_inv.png")
    os.makedirs(os.path.join(MODELS, "cross_marker_col"), exist_ok=True)
    r2 = make_marker_png(os.path.join(SRC_MODEL, "cross_marker.png"),
                         os.path.join(MODELS, "cross_marker_col", "cross_marker_col.png"), "col")
    make_model("cross_marker_col", "cross_marker_col.png")
    print(f"  inverted marker  plate lum {r1['plate']:.0f}  cross lum {r1['cross']:.0f}")
    print(f"  coloured marker  plate lum {r2['plate']:.0f}  cross lum {r2['cross']:.0f}  (delta {abs(r2['plate']-r2['cross']):.0f})")
    made = [
        make_world("cm_dim",    light_intensity=0.30),
        make_world("cm_bright", light_intensity=2.50),
        make_world("cm_lowsun", light_dir="0.70 0.30 -0.20"),   # grazing -> long hard shadows
        make_world("cm_inv",    marker_model="cross_marker_inv"),
        make_world("cm_col",    marker_model="cross_marker_col"),
        make_world("cm_darkbg", ground_rgb="0.18 0.18 0.18 1"), # dark ground: global-darkening case
    ]
    for p in made: print("  world:", os.path.basename(p))

if __name__ == "__main__":
    main()
