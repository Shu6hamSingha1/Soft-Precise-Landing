# HARDWARE COPY (2026-09-24) of PX4_Gazebo/src/cross_stroke_detector.py @ 1eaffe4, unmodified below this
# header. Keep in sync with the PX4 source. Selected via CROSS_DETECTOR=stroke (cross_marker_detector.detect dispatches).
"""Stroke-based cross-marker detector -- the LOCKED DESIGN front end (2026-09-24).

Design (Memory/px4/feedback_cross_detector_robustness_requirement.md, user requirement
2026-09-02): robust to lighting, marker/background colour, texture and perception noise;
no absolute intensity gate; no mask-centroid junction proxy.

  1. STROKE RESPONSE -- multi-scale Hessian ridge measure on luminance. Polarity-agnostic
     (|lambda1|: dark-on-light and light-on-dark respond identically), relative (normalised
     derivatives, thresholds relative to the frame's own strongest ridges), scale-free
     (strokes span ~1.8 px at 5 m to ~60 px at 0.15 m; the scale set covers it). A STEP edge
     (plate outline, shadow boundary, platform side) is suppressed by a gradient-symmetry
     term: at a ridge centre the gradient vanishes, at a step it peaks.
  2. CENTRELINES -- non-maximum suppression across the stroke (along the Hessian normal).
  3. LINE HYPOTHESES -- orientation-consistent Hough over centreline pixels, then a weighted
     TLS refit per line on its inliers.
  4. X-JUNCTION CONFIRM (geometry-first) -- choose the line pair that CROSSES: ridge support
     on BOTH sides of the intersection along BOTH lines, with matching stroke widths. A plate
     corner / platform L (two strokes MEETING, the rover_cross false lock) has one-sided
     support and loses by construction. No mask centroid anywhere.
  5. STUB -- a third ridge line through the junction, one-sided, gives the heading.

Returns cross_marker_detector.CrossMarkerDetection so callers are unchanged; enabled with
CROSS_DETECTOR=stroke (dispatch in cross_marker_detector.detect).
"""
import os
import numpy as np
import cv2

# ---- parameters (all scale-free / relative) ----
SIGMAS = tuple(float(s) for s in os.environ.get(
    "CROSS_STROKE_SIGMAS", "0.8,1.2,1.8,2.7,4,6,9,13.5,20,30,40").split(","))
# 30/40 (2026-09-24): below ~0.3 m the stroke is 60-80 px wide; with sigma<=20 the band resolves
# as two EDGES and adjacent arms meet in L-corners -> honest X-test refusals on ~50% of frames.
SYM_K = float(os.environ.get("CROSS_STROKE_SYM_K", "1.0"))          # step-edge suppression weight
REL_THR = float(os.environ.get("CROSS_STROKE_REL_THR", "0.25"))     # centreline keep: S > REL_THR * S_p99.5
MIN_CONTRAST = float(os.environ.get("CROSS_STROKE_MIN_CONTRAST", "0.04"))  # absolute floor on S (frac of full scale)
N_LINES = int(os.environ.get("CROSS_STROKE_N_LINES", "8"))
ORI_TOL_DEG = float(os.environ.get("CROSS_STROKE_ORI_TOL_DEG", "12"))
PAIR_MIN_DEG, PAIR_MAX_DEG = 50.0, 130.0
BAL_MIN = float(os.environ.get("CROSS_STROKE_BAL_MIN", "0.3"))       # min bilateral balance per line
WIDTH_RATIO_MAX = float(os.environ.get("CROSS_STROKE_WIDTH_RATIO_MAX", "1.8"))
DEBUG = {}


def _ridge(gray, sigmas=None):
    """Best-over-scales ridge score S, scale sigma* (full-res px), unit normal (nx, ny),
    polarity sign. Large scales run on a pyramid level where the effective sigma is
    1.35-2.7 px; scales sharing a level are max-combined THERE and upsampled once."""
    H, W = gray.shape
    sigmas = SIGMAS if sigmas is None else sigmas
    pyr = [gray]
    by_lev = {}
    for s in sigmas:
        if s > min(H, W) / 5:        # stroke wider than ~40% of the short side: nothing to resolve
            break
        lev = 0
        while s / (2 ** lev) > 2.7 and min(pyr[-1].shape) > 24:
            lev += 1
            if len(pyr) <= lev:
                pyr.append(cv2.pyrDown(pyr[-1]))
        by_lev.setdefault(min(lev, len(pyr) - 1), []).append(s)
    S = np.full((H, W), -np.inf, np.float32)
    SIG = np.zeros((H, W), np.float32)
    NX = np.zeros((H, W), np.float32); NY = np.zeros((H, W), np.float32)
    POL = np.zeros((H, W), np.int8)
    for lev, ss in sorted(by_lev.items()):
        img = pyr[lev]
        Sl = np.full(img.shape, -np.inf, np.float32); Gl = np.zeros(img.shape, np.float32)
        Xl = np.zeros(img.shape, np.float32); Yl = np.zeros(img.shape, np.float32)
        Pl = np.zeros(img.shape, np.int8)
        for s in ss:
            Sc, vx, vy, pol = _ridge_one(img, s / (2 ** lev))
            u = Sc > Sl
            np.copyto(Sl, Sc, where=u); Gl[u] = s                 # copyto: boolean-index assign was ~4 ms/frame
            np.copyto(Xl, vx, where=u); np.copyto(Yl, vy, where=u); np.copyto(Pl, pol, where=u)
        if lev:
            Sl = cv2.resize(Sl, (W, H), interpolation=cv2.INTER_LINEAR)
            Gl = cv2.resize(Gl, (W, H), interpolation=cv2.INTER_NEAREST)
            Xl = cv2.resize(Xl, (W, H), interpolation=cv2.INTER_LINEAR)
            Yl = cv2.resize(Yl, (W, H), interpolation=cv2.INTER_LINEAR)
            Pl = cv2.resize(Pl, (W, H), interpolation=cv2.INTER_NEAREST)
            nrm = np.sqrt(Xl * Xl + Yl * Yl) + 1e-12
            Xl /= nrm; Yl /= nrm
        upd = Sl > S
        np.copyto(S, Sl, where=upd); np.copyto(SIG, Gl, where=upd)
        np.copyto(NX, Xl, where=upd); np.copyto(NY, Yl, where=upd)
        POL[upd] = Pl[upd]
    return S, SIG, NX, NY, POL


_GRID_CACHE = {}


def _grids(H, W):
    """Shape-cached (yy, xx, distance-to-frame-edge) -- rebuilt only when the crop size changes."""
    k = (H, W)
    if k not in _GRID_CACHE:
        if len(_GRID_CACHE) > 64:
            _GRID_CACHE.clear()
        yy, xx = np.mgrid[0:H, 0:W]
        _GRID_CACHE[k] = (yy, xx, np.minimum(np.minimum(xx, W - 1 - xx), np.minimum(yy, H - 1 - yy)))
    return _GRID_CACHE[k]


def _ridge_one(gray, s):
    """Normalised ridge score at one scale (pixel units of `gray`)."""
    kxx = np.array([[1, -2, 1]], np.float32)
    kd = np.array([[-0.5, 0, 0.5]], np.float32)
    if True:
        G = cv2.GaussianBlur(gray, (0, 0), s, borderType=cv2.BORDER_REPLICATE)
        Ixx = cv2.filter2D(G, -1, kxx, borderType=cv2.BORDER_REPLICATE)
        Iyy = cv2.filter2D(G, -1, kxx.T, borderType=cv2.BORDER_REPLICATE)
        Gx = cv2.filter2D(G, -1, kd, borderType=cv2.BORDER_REPLICATE)
        Gy = cv2.filter2D(G, -1, kd.T, borderType=cv2.BORDER_REPLICATE)
        Ixy = cv2.filter2D(Gx, -1, kd.T, borderType=cv2.BORDER_REPLICATE)
        tr = 0.5 * (Ixx + Iyy)
        dd = np.sqrt(0.25 * (Ixx - Iyy) ** 2 + Ixy ** 2)
        l_a, l_b = tr + dd, tr - dd
        big = np.abs(l_a) >= np.abs(l_b)
        l1 = np.where(big, l_a, l_b); l2 = np.where(big, l_b, l_a)
        # normal = eigenvector of l1
        vx = Ixy; vy = l1 - Ixx
        alt = (np.abs(vx) + np.abs(vy)) < 1e-9
        vx = np.where(alt, l1 - Iyy, vx); vy = np.where(alt, Ixy, vy)
        nrm = np.sqrt(vx * vx + vy * vy) + 1e-12
        vx /= nrm; vy /= nrm
        Sc = (s * s) * (np.abs(l1) - np.abs(l2)) - SYM_K * s * np.sqrt(Gx * Gx + Gy * Gy)
        pol = np.where(l1 > 0, 1, -1).astype(np.int8)   # +1 dark stroke (valley), -1 bright stroke
    return Sc.astype(np.float32), vx.astype(np.float32), vy.astype(np.float32), pol


def _centrelines(S, NX, NY):
    """Non-max suppression across the stroke; returns boolean map."""
    H, W = S.shape
    ys, xs = np.mgrid[0:H, 0:W].astype(np.float32)
    s1 = cv2.remap(S, xs + NX, ys + NY, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    s2 = cv2.remap(S, xs - NX, ys - NY, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    return (S >= s1) & (S >= s2)


def _hough(xs, ys, th_pt, w, H, W, n_lines):
    """Orientation-consistent Hough: each point votes only near its own line direction.
    Line: x cos t + y sin t = rho, t in [0, 180) deg."""
    diag = int(np.ceil(np.hypot(H, W)))
    nt, rstep = 180, 2.0
    nr = int(2 * diag / rstep) + 1
    acc = np.zeros((nt, nr), np.float64)
    tol = int(ORI_TOL_DEG)
    for dt in range(-tol, tol + 1, 2):
        t = (th_pt + dt) % 180
        tr = np.radians(t)
        rho = xs * np.cos(tr) + ys * np.sin(tr)
        ri = np.round((rho + diag) / rstep).astype(int)
        np.add.at(acc, (t.astype(int) % 180, np.clip(ri, 0, nr - 1)), w)
    acc = cv2.GaussianBlur(acc.astype(np.float32), (5, 5), 1.0)
    peaks = []
    a = acc.copy()
    for _ in range(n_lines):
        k = int(np.argmax(a))
        ti, ri = divmod(k, nr)
        if a[ti, ri] <= 0:
            break
        peaks.append((float(ti), ri * rstep - diag, float(a[ti, ri])))
        # suppress neighbourhood (wrap theta)
        for dti in range(-8, 9):
            a[(ti + dti) % nt, max(0, ri - 6):ri + 7] = 0
    return peaks


def _fit_line(px, py, w):
    """Weighted TLS line: returns point (mx, my) and unit direction (dx, dy)."""
    ws = w / (w.sum() + 1e-12)
    mx, my = float((px * ws).sum()), float((py * ws).sum())
    cx, cy = px - mx, py - my
    C = np.array([[np.sum(ws * cx * cx), np.sum(ws * cx * cy)],
                  [np.sum(ws * cx * cy), np.sum(ws * cy * cy)]])
    ev, evec = np.linalg.eigh(C)
    d = evec[:, 1]
    return mx, my, float(d[0]), float(d[1])


def _intersect(a, b):
    (ax, ay, adx, ady), (bx, by, bdx, bdy) = a, b
    den = adx * bdy - ady * bdx
    if abs(den) < 1e-6:
        return None
    t = ((bx - ax) * bdy - (by - ay) * bdx) / den
    return ax + t * adx, ay + t * ady


TRACK_MAX_MISSES = int(os.environ.get("CROSS_STROKE_TRACK_MAX_MISSES", "3"))
WORKRES_H = float(os.environ.get("CROSS_STROKE_WORKRES_H", "5"))    # tracked stroke sigma >= this: half res
WORKRES_Q = float(os.environ.get("CROSS_STROKE_WORKRES_Q", "12"))   # >= this: quarter res


def _channel(frame_bgr, ch, cache):
    """Float [0,1] image for channel 'L' (luminance), 'a' or 'b' (Lab chroma)."""
    if ch == "L":
        return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
    if "lab" not in cache:
        cache["lab"] = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2Lab)
    return cache["lab"][:, :, "Lab".index(ch)].astype(np.float32) / 255.0


CHANNELS = tuple(c for c in os.environ.get("CROSS_STROKE_CHANNELS", "L,a,b").split(","))


def detect_stroke(frame_bgr, track_state=None):
    """CHANNEL CASCADE (colour-agnostic): luminance first; if it yields no confirmed X, retry on
    the Lab chroma channels. A red-on-green iso-V marker (cm_col) has ~0.15 luminance contrast
    but ~0.45 in `a`. The X-junction test stays the arbiter on every channel, so a chroma
    channel can only ADD a verified X, never lower the bar. The winning channel is remembered
    in track_state and tried first next frame (clean scenes never pay for chroma)."""
    ts = track_state if track_state is not None else {}
    pref = ts.get("stroke_chan", CHANNELS[0])
    order = [pref] + [c for c in CHANNELS if c != pref]
    cache, first = {}, None
    for k, ch in enumerate(order):
        det = _detect_on(_channel(frame_bgr, ch, cache), ts if k == 0 else {})
        if det.ok:
            if k:                                # fallback channel won: seed its track
                ts.update(stroke_bbox=det.mask_bbox, stroke_w=_detect_on.last_w, stroke_miss=0)
            ts["stroke_chan"] = ch
            return det
        first = first or det
    return first


def _detect_on(gray, ts):
    """Tracked fast path (same shape as cross_marker_detector.detect's ROI lock): with a
    recent lock, search only last bbox + margin and only scales within 1.8x of the last
    stroke width; any miss falls through to the full-frame, all-scale search."""
    from cross_marker_detector import CrossMarkerDetection
    from dataclasses import replace
    H, W = gray.shape
    bb, w0 = ts.get("stroke_bbox"), ts.get("stroke_w")
    if bb is not None and w0 and ts.get("stroke_miss", 0) < TRACK_MAX_MISSES:
        bx, by, bw, bh = bb
        m = 0.5 * max(bw, bh) + 4.0 * w0 + 8
        x0, y0 = int(max(0, bx - m)), int(max(0, by - m))
        x1, y1 = int(min(W, bx + bw + m)), int(min(H, by + bh + m))
        sig = tuple(s for s in SIGMAS if w0 / 1.8 <= s <= w0 * 1.8) or SIGMAS
        # WORKING RESOLUTION matched to the stroke: a stroke >= 5 px wide loses nothing at half
        # resolution (>= 10 px: quarter), and that is exactly when the ROI is large (mid/low
        # altitude). Same principle as cross_marker_detector's DETECT_WORK_MAX_PX cap.
        f = 4 if w0 >= WORKRES_Q else (2 if w0 >= WORKRES_H else 1)
        if (x1 - x0) // f > 24 and (y1 - y0) // f > 24:
            crop = gray[y0:y1, x0:x1]
            if f > 1:
                crop = cv2.resize(crop, ((x1 - x0) // f, (y1 - y0) // f), interpolation=cv2.INTER_AREA)
                sig_f = tuple(s / f for s in sig if s / f >= 0.8) or (0.8,)
            else:
                sig_f = sig
            det, wd = _core(crop, sig_f, CrossMarkerDetection)
            if det.ok and wd is not None:
                wd = wd * f
            if det.ok:
                # crop px -> full-frame px: p_full = (p + 0.5) * f - 0.5 + offset (INTER_AREA centres)
                g = lambda v, o: (v + 0.5) * f - 0.5 + o
                sh = lambda P: tuple((g(px, x0), g(py, y0)) for px, py in P) if P else P
                mask = np.zeros((H, W), np.uint8)
                if det.isolated_mask is not None:
                    m_ = det.isolated_mask
                    if f > 1:
                        m_ = cv2.resize(m_, (x1 - x0, y1 - y0), interpolation=cv2.INTER_NEAREST)
                    mask[y0:y0 + m_.shape[0], x0:x0 + m_.shape[1]] = m_
                c = (g(det.center[0], x0), g(det.center[1], y0))
                det = replace(det, center=c,
                              mask_bbox=(int(det.mask_bbox[0] * f + x0), int(det.mask_bbox[1] * f + y0),
                                         int(det.mask_bbox[2] * f), int(det.mask_bbox[3] * f)),
                              line_points_i=sh(det.line_points_i), line_points_j=sh(det.line_points_j),
                              line_points_i_raw=sh(det.line_points_i_raw),
                              line_points_j_raw=sh(det.line_points_j_raw),
                              stub_points=sh(det.stub_points), isolated_mask=mask,
                              in_fov=(0 <= c[0] < W) and (0 <= c[1] < H))
                ts.update(stroke_bbox=det.mask_bbox, stroke_w=wd, stroke_miss=0)
                _detect_on.last_w = wd
                return det
        ts["stroke_miss"] = ts.get("stroke_miss", 0) + 1
    det, wd = _core(gray, SIGMAS, CrossMarkerDetection)
    if det.ok:
        ts.update(stroke_bbox=det.mask_bbox, stroke_w=wd, stroke_miss=0)
    _detect_on.last_w = wd
    return det


RING_MIN_X = int(os.environ.get("CROSS_STROKE_RING_MIN_X", "6"))


def _ring_x(gray, J, width, pol):
    """Stroke crossings on a ring around a candidate junction -- the TOPOLOGY confirm used when
    bilateral centreline support is inconclusive (terminal range: the arms merge into one dark
    blob at the junction, leaving a line one-sided inside the frame). An X crosses the ring 4
    arms + stub = ~10 transitions; a platform L 4; a plate corner 2. Radius 2.5 stroke widths
    (outside the blob), stroke/plate split = Otsu on the ring samples themselves (relative,
    polarity from the lines). Returns transitions over the in-frame arc, or None if too little
    of the ring is visible to judge."""
    H, W = gray.shape
    r = 2.5 * max(width, 2.0)
    th = np.linspace(0, 2 * np.pi, 240, endpoint=False)
    xs = J[0] + r * np.cos(th); ys = J[1] + r * np.sin(th)
    ok = (xs >= 1) & (xs < W - 2) & (ys >= 1) & (ys < H - 2)
    if ok.mean() < 0.6:
        return None
    v = cv2.remap(gray, xs[ok].astype(np.float32)[None, :], ys[ok].astype(np.float32)[None, :],
                  cv2.INTER_LINEAR)[0]
    if float(v.max() - v.min()) < 0.05:
        return 0
    u8 = np.clip((v - v.min()) / (v.max() - v.min()) * 255, 0, 255).astype(np.uint8)
    t, _ = cv2.threshold(u8, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    stroke = (u8 <= t) if pol > 0 else (u8 > t)        # +1 = dark strokes
    # contiguous in-frame arc only: count transitions along it (no wrap across the gap)
    if ok.all():
        return int(np.sum(stroke != np.roll(stroke, 1)))
    return int(np.sum(stroke[1:] != stroke[:-1]))


def _core(gray, sigmas, CrossMarkerDetection):
    """Full stroke detection on one grey image (or crop). Returns (det, stroke_width_px)."""
    DEBUG.clear()
    H, W = gray.shape
    S, SIG, NX, NY, POL = _ridge(gray, sigmas)
    cl = _centrelines(S, NX, NY)
    # BORDER: filtering uses BORDER_REPLICATE -- BORDER_REFLECT mirrored a step touching the
    # frame edge into a symmetric bar = a false ridge ALONG the edge (inv: two such lines met at
    # the frame corner and beat the real X; col: a bright top strip set the relative threshold).
    # A 2*sigma exclusion fixed that but blanked the sigma 30-40 scales the <0.3 m strokes need
    # (60-80 px of a 240 px frame); replicate removes the artifact at source, so only a fixed
    # few-pixel margin remains.
    yy, xx, edge_d = _grids(H, W)
    cl &= edge_d > 3
    S = np.where(edge_d > 3, S, -np.inf).astype(np.float32)
    pos = S[np.isfinite(S) & (S > 0)]
    if pos.size < 20:
        return CrossMarkerDetection(None, None, False, None, fail_reason="stroke_no_ridges"), None
    thr = max(REL_THR * float(np.percentile(pos, 99.5)), MIN_CONTRAST)
    keep = cl & (S > thr)
    ys, xs = np.nonzero(keep)
    if len(xs) < 20:
        return CrossMarkerDetection(None, None, False, None, fail_reason="stroke_few_centreline_px"), None
    w = S[ys, xs].astype(np.float64)
    sig = SIG[ys, xs].astype(np.float64)
    pol = POL[ys, xs]
    # line direction = perpendicular to normal; Hough theta = normal angle
    th_pt = (np.degrees(np.arctan2(NY[ys, xs], NX[ys, xs])) % 180).astype(np.float64)
    xs = xs.astype(np.float64); ys = ys.astype(np.float64)
    DEBUG.update(S=S, keep=keep, thr=thr)
    peaks = _hough(xs, ys, th_pt, w, H, W, N_LINES)
    lines = []
    for t_deg, rho, score in peaks:
        tr = np.radians(t_deg)
        d = np.abs(xs * np.cos(tr) + ys * np.sin(tr) - rho)
        dori = np.abs(((th_pt - t_deg) + 90) % 180 - 90)
        # seed with a generous band around the COARSE Hough peak (1 deg bins -> 2.6 px error
        # 150 px out), then refit + re-collect along the REFINED line with the tight
        # tolerance. Collecting straight off the coarse peak split one diagonal into several
        # one-sided segments (base 0.7-1.3 m band, detOK 53%).
        inl = (d <= 6.0) & (dori <= ORI_TOL_DEG + 4)
        if inl.sum() < 8:
            continue
        # one polarity per line (the stroke's own)
        p_major = 1 if (pol[inl] > 0).sum() >= (pol[inl] < 0).sum() else -1
        inl &= (pol == p_major)
        if inl.sum() < 8:
            continue
        for _ in range(2):
            mx, my, dx, dy = _fit_line(xs[inl], ys[inl], w[inl])
            dl = np.abs((xs - mx) * dy - (ys - my) * dx)
            th_l = np.degrees(np.arctan2(dx, -dy)) % 180          # normal angle of the refit line
            dori = np.abs(((th_pt - th_l) + 90) % 180 - 90)
            # positional tolerance: sigma is the stroke SCALE, not the centreline's position
            # error -- a 1.5*sigma band (30 px at close range) swallowed band edges + texture
            new = (dl <= np.maximum(2.0, 0.25 * sig)) & (dori <= ORI_TOL_DEG) & (pol == p_major)
            if new.sum() < 8:
                break
            inl = new
        if inl.sum() < 8:
            continue
        L = _fit_line(xs[inl], ys[inl], w[inl])
        # drop duplicates of an already-accepted line (same direction, same offset)
        dup = False
        for ln in lines:
            (ax, ay, adx, ady) = ln["L"]
            if abs(adx * L[2] + ady * L[3]) > np.cos(np.radians(3.0)) and \
               abs((L[0] - ax) * ady - (L[1] - ay) * adx) < max(3.0, ln["width"]):
                dup = True
                break
        if dup:
            continue
        lines.append(dict(L=L, idx=np.nonzero(inl)[0], pol=p_major,
                          width=float(np.median(sig[inl])), strength=float(w[inl].sum())))
    DEBUG["lines"] = lines
    if len(lines) < 2:
        return CrossMarkerDetection(None, None, False, None, fail_reason="stroke_lt2_lines"), None

    def side_stats(ln, J):
        mx, my, dx, dy = ln["L"]
        i = ln["idx"]
        t = (xs[i] - J[0]) * dx + (ys[i] - J[1]) * dy
        gap = 2.0 * ln["width"]
        lo, hi = t[t < -gap], t[t > gap]
        span_lo = float(-lo.min()) if lo.size else 0.0
        span_hi = float(hi.max()) if hi.size else 0.0
        # available length to the frame edge on each side (clipping -> not a veto)
        def avail(sgn):
            ts = []
            for (c, dc, lim) in ((J[0], sgn * dx, W - 1), (J[1], sgn * dy, H - 1)):
                if dc > 1e-9: ts.append((lim - c) / dc)
                elif dc < -1e-9: ts.append((0 - c) / dc)
            return max(0.0, min(ts)) if ts else 1e9
        a_lo, a_hi = avail(-1), avail(1)
        # A side too short to hold a verifiable arm (junction at/near the frame edge) is
        # UNVERIFIABLE, not balanced -- granting it balance let two border lines meeting at the
        # frame corner pass as an X. Such a line returns bal=None; the pair needs >=1 line
        # with verified bilateral support.
        vmin = max(2.0 * ln["width"], 8.0)   # was 4w: at sigma 30-40 that is 120-160 px = the whole
                                              # junction-to-edge distance -> terminal refusals
        if a_lo < vmin or a_hi < vmin:
            return None, span_lo, span_hi, t
        # a side that runs into the frame edge is "at least" what is visible
        c_lo = span_lo >= a_lo - 3.0 * ln["width"] - 2
        c_hi = span_hi >= a_hi - 3.0 * ln["width"] - 2
        if c_lo and c_hi:
            bal = 1.0
        elif c_lo:
            bal = min(1.0, span_hi / max(min(span_lo, a_lo), 1e-6))
        elif c_hi:
            bal = min(1.0, span_lo / max(min(span_hi, a_hi), 1e-6))
        else:
            bal = min(span_lo, span_hi) / max(span_lo, span_hi, 1e-6)
        return bal, span_lo, span_hi, t

    best = None
    for a in range(len(lines)):
        for b in range(a + 1, len(lines)):
            la, lb = lines[a], lines[b]
            if la["pol"] != lb["pol"]:
                continue
            # acute angle between the two undirected lines; X arms are ~90 deg on the plate,
            # perspective/tilt keeps them inside [PAIR_MIN_DEG, 180 - PAIR_MIN_DEG]
            ang = np.degrees(np.arccos(np.clip(abs(la["L"][2] * lb["L"][2] + la["L"][3] * lb["L"][3]), 0, 1)))
            if ang < min(PAIR_MIN_DEG, 180.0 - PAIR_MAX_DEG):
                continue
            wr = max(la["width"], lb["width"]) / max(min(la["width"], lb["width"]), 1e-6)
            if wr > WIDTH_RATIO_MAX:
                continue
            J = _intersect(la["L"], lb["L"])
            if J is None or not (-0.5 * W <= J[0] <= 1.5 * W and -0.5 * H <= J[1] <= 1.5 * H):
                continue
            ba, *_ = side_stats(la, J)
            bb, *_ = side_stats(lb, J)
            if ba is None and bb is None:
                continue                      # neither line can verify an X here
            ba = bb if ba is None else ba
            bb = ba if bb is None else bb
            bal = min(ba, bb)
            if bal < BAL_MIN and 0 <= J[0] < W and 0 <= J[1] < H:
                nx = _ring_x(gray, J, 0.5 * (la["width"] + lb["width"]), la["pol"])
                if nx is not None and nx >= RING_MIN_X:
                    bal = max(bal, BAL_MIN)      # topology confirms the X; balance was the blob
            score = bal * np.sqrt(la["strength"] * lb["strength"])
            if best is None or score > best[0]:
                best = (score, a, b, J, bal, bal, wr)
    if best is None:
        return CrossMarkerDetection(None, None, False, None, fail_reason="stroke_no_crossing_pair"), None
    score, a, b, J, ba, bb, wr = best
    DEBUG["best"] = best
    if min(ba, bb) < BAL_MIN:
        return CrossMarkerDetection(None, None, False, None, fail_reason="stroke_not_x_junction"), None
    la, lb = lines[a], lines[b]
    in_fov = (0 <= J[0] < W) and (0 <= J[1] < H)
    pi = tuple(zip(xs[la["idx"]].tolist(), ys[la["idx"]].tolist()))
    pj = tuple(zip(xs[lb["idx"]].tolist(), ys[lb["idx"]].tolist()))
    # stub: another same-polarity line through J (within ~2 widths), roughly bisecting, one-sided
    stub_pts, heading = None, None
    for k, ln in enumerate(lines):
        if k in (a, b) or ln["pol"] != la["pol"]:
            continue
        mx, my, dx, dy = ln["L"]
        dist = abs((J[0] - mx) * dy - (J[1] - my) * dx)
        if dist > 2.5 * max(ln["width"], 1.0):
            continue
        bal, s_lo, s_hi, t = side_stats(ln, J)
        if bal is not None and bal > 0.35:
            continue
        sgn = 1 if s_hi > s_lo else -1
        sel = ln["idx"][(t * sgn) > 2.0 * ln["width"]]
        if len(sel) >= 4:
            stub_pts = tuple(zip(xs[sel].tolist(), ys[sel].tolist()))
            heading = float(np.degrees(np.arctan2(sgn * dy, sgn * dx)))
            break
    allp = np.array(pi + pj + (stub_pts or ()), float)
    x0, y0 = allp.min(axis=0); x1, y1 = allp.max(axis=0)
    bbox = (int(x0), int(y0), int(max(1, x1 - x0)), int(max(1, y1 - y0)))
    # stroke-pixel mask (polarity-agnostic): near a fitted arm AND closer to the stroke's own
    # intensity than to the local plate level
    mask = np.zeros((H, W), np.uint8)
    # only the arms' own bounding region (+ the 3*hw plate ring) -- full-frame distance maps per
    # line were ~2 ms/frame
    for ln in (la, lb):
        mx, my, dx, dy = ln["L"]
        hw = max(2.0, 2.0 * ln["width"])
        P = np.array(pi if ln is la else pj, float)
        r0 = int(max(0, P[:, 1].min() - 3.5 * hw)); r1 = int(min(H, P[:, 1].max() + 3.5 * hw + 1))
        c0 = int(max(0, P[:, 0].min() - 3.5 * hw)); c1 = int(min(W, P[:, 0].max() + 3.5 * hw + 1))
        gy, gx = yy[r0:r1, c0:c1], xx[r0:r1, c0:c1]
        dperp = np.abs((gx - mx) * dy - (gy - my) * dx)
        band = dperp <= hw * 1.5
        core = dperp <= 1.0
        ring = (dperp >= hw * 2.0) & (dperp <= hw * 3.0)
        gsub = gray[r0:r1, c0:c1]
        if core.any() and ring.any():
            s_lvl = float(np.median(gsub[core & band])); p_lvl = float(np.median(gsub[ring]))
            sub = mask[r0:r1, c0:c1]
            sub[band & (np.abs(gsub - s_lvl) < np.abs(gsub - p_lvl))] = 255
    return CrossMarkerDetection(center=(float(J[0]), float(J[1])), heading_deg=heading, ok=True,
                                mask_bbox=bbox, fail_reason=None,
                                line_points_i=pi, line_points_j=pj, stub_points=stub_pts,
                                line_points_i_raw=pi, line_points_j_raw=pj,
                                isolated_mask=mask, in_fov=in_fov), 0.5 * (la["width"] + lb["width"])
