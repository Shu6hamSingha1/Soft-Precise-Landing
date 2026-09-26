"""Image-only oracle for the cross marker: >=2 thin dark stroke lines (distinct angles, meeting near each other)
on the bright paper, partial visibility allowed. Independent of the detectors under test."""
import cv2, numpy as np

def oracle(gray):
    g = cv2.GaussianBlur(gray, (3, 3), 0)
    ctx = cv2.blur(g, (15, 15))
    paper_zone = ctx > max(np.median(g) + 18, np.percentile(g, 85))       # bright paper neighbourhood
    bh = cv2.morphologyEx(g, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    m = ((bh > 11) & paper_zone).astype(np.uint8) * 255
    segs = cv2.HoughLinesP(m, 1, np.pi / 180, 7, minLineLength=9, maxLineGap=6)
    if segs is None: return dict(vis=False, n=0)
    segs = np.asarray(segs, float).reshape(-1, 4)
    S = [(s, (np.degrees(np.arctan2(s[3] - s[1], s[2] - s[0])) % 180), np.hypot(s[2] - s[0], s[3] - s[1])) for s in segs]
    S.sort(key=lambda t: -t[2])
    best = None
    for i in range(len(S)):
        for j in range(i + 1, len(S)):
            (a, aa, la), (b, ab, lb) = S[i], S[j]
            da = abs((aa - ab + 90) % 180 - 90)
            if da < 25: continue
            A = np.array([[a[2] - a[0], -(b[2] - b[0])], [a[3] - a[1], -(b[3] - b[1])]], float)
            if abs(np.linalg.det(A)) < 1e-6: continue
            t = np.linalg.solve(A, [b[0] - a[0], b[1] - a[1]])
            px, py = a[0] + t[0] * (a[2] - a[0]), a[1] + t[0] * (a[3] - a[1])
            def dseg(s):   # distance point->segment
                p = np.array([px, py]); p0 = np.array(s[:2], float); p1 = np.array(s[2:], float); d = p1 - p0
                u = np.clip(((p - p0) @ d) / (d @ d), 0, 1); return np.linalg.norm(p - (p0 + u * d))
            if dseg(a) < 25 and dseg(b) < 25:
                sc = la + lb
                if best is None or sc > best[0]: best = (sc, px, py)
        if best: break
    if best is None: return dict(vis=False, n=len(S))
    return dict(vis=True, n=len(S), cx=float(best[1]), cy=float(best[2]))
