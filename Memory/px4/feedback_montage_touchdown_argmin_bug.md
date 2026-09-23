---
name: feedback_montage_touchdown_argmin_bug
description: "make_landing_montage.py's load_series() falsely used argmin(uz) as touchdown for runs that NEVER landed, truncating plots+video up to ~10s early -- fixed by gating the trim on Ground_Truth.npy's own SoftPrecise.xy_err."
metadata:
  type: project
  modified: 2026-09-23T03:35:00.000Z
---

2026-09-23, user-reported: "the plots and the chase cam feed doesn't match for some
test runs of the baselines especially for FF-IBVS" (cho2022). Root-caused and fixed
same session -- see [[project_20260923_comparative_baseline_campaign]] for the
campaign this surfaced in and [[project_20260923_manuscript_video_symbol_convention]]
for the unrelated symbol-convention work done just before this in the same file.

**Bug:** `tools/make_landing_montage.py::load_series()` found "touchdown" via
`itd = argmin(uz) + 1`, then trimmed EVERY series (position/velocity plots, AND the
montage's own frame count, since `dur = series["t"][-1]` drives it) to that index.
Correct when the run actually lands. Wrong when it doesn't: a stalled/aborted run
(e.g. cho2022, which hits an identical `descent stall` abort on ALL 10/10 recorded
cases -- see the campaign memory) just hovers near a flat altitude with mm-scale
sensor noise for the rest of the recording; argmin locks onto an arbitrary noise dip
mid-hover, not a real event. Concretely (CHO2022-GT/IC1): hovers flat ~0.497m from
t=8.9s to the true end at t=31.7s; argmin picked t=22.1s, discarding the last 9.6s of
real, still-recorded flight from BOTH the plots and the composited video. Montage
duration was 23.04s against a 33.3s real chase video.

**Diagnosis method:** computed `itd/n` (touchdown-index fraction) across every
Final/<TAG>-GT/<case> Ground_Truth.npy. Genuine landings cluster at 0.95-1.0 (VISTA-GT:
0.993-0.999). Aborted runs vary widely (0.27-0.88) since the noise-driven argmin has no
relationship to recording length. **This fraction alone is not a reliable landed/
aborted classifier** -- some genuinely-landed runs (LIN2023-GT/Circular) also show
~0.96 from a real post-touchdown bounce, and some aborted runs (LIN2022-GT/IC2/IC3/
IC5/Static) coincidentally scored >0.97 (noise dip happened to land near the true end
anyway) despite being genuinely unlanded. The RELIABLE signal is
`Ground_Truth.npy['SoftPrecise'].get('xy_err') is not None` -- same landing-detection
logic the harness itself uses to populate that dict; empty `{}` = never reached
touchdown detection at all.

**Fix:** `load_series()` now checks that `xy_err` signal FIRST. If landed, same
argmin-trim as before (unchanged behavior for the common case). If not landed, `itd =
n` (full untrimmed series) with a printed WARNING so it's visible in batch-regen logs.

**Distinct, NOT-a-bug phenomenon found during the same investigation:** several
genuinely-landed runs (LIN2022-GT IC1/IC4/Sinusoidal, ZHANG2026-GT IC1/Lissajous,
LIN2023-GT/Circular) show a real altitude BOUNCE after touchdown (e.g. LIN2022-GT/IC4:
touches down at t=14.9s, then bounces/moves for another 10.5s up to 0.63m) that the
argmin-trim correctly excludes by design (same crop-at-first-touchdown convention
VISTA-GT already uses) -- don't conflate this with the argmin bug; these were left
untouched.

**Regeneration fallout, a second real bug found while fixing the first:**
`LIN2022-GT/IC3`'s original campaign rep never saved an `onboard_cam.mp4` at all
(likely a per-rep recording failure in that specific run, unrelated to this montage
tool). Regenerating its montage against a missing/empty onboard file silently produced
a 0-frame drone PiP (`drone 0f td@1675 (+-1676f tail)` in the tool's own diagnostic
line -- always read that line, don't just check the process exit code). Caught by
grep'ing both batch logs for `drone 0f` before copying anything over; the broken file
was deleted rather than committed. **Lesson: when batch-regenerating from a tool with
a diagnostic summary line, grep every log for the anomaly signature before trusting
"the loop completed" as "every output is valid."**

**Regenerated (17 of the 19 never-landed cases; LIN2022-GT/IC3 dropped, LIN2022-GT/
Static was byte-identical so git saw no diff):** CHO2022-GT's full 10, plus LIN2022-GT
IC2/IC5, LIN2023-GT IC2/IC3/IC4/IC5/Static. Duration deltas ranged from negligible
(~0.1s, cases where the noise-argmin already landed near the true end) to severe
(CHO2022-GT/Circular: 24.6s -> 37.0s). Committed `310c50c5`.

Fix is in the SHARED tool (`tools/make_landing_montage.py`) -- applies automatically to
any future montage regen, VISTA-GT included, not baseline-specific.

**Follow-up (2026-09-23, night): LIN2022-GT/IC3 re-recorded, gap closed.** Re-ran
`record_baseline_cases.sh lin2022 IC3` once; same descent-stall abort (consistent with the
original), and onboard + chase videos were captured this time, so the folder now has a valid
montage. Two operational lessons from getting there: (1) my first attempt used a foreground
`timeout 110` -- an aborting run takes ~4 min, so it was killed mid-flight; run SITL
recordings in the background. (2) my cleanup used a broad `pkill -f` (px4 / gz sim /
MicroXRCEAgent / landing_test) that killed my OWN shell and probably hit a concurrent
session's in-flight SITL run (`RoverIC_raw/record_rover_ic_raw.sh`) -- kill by exact PID
(`pgrep -x`), never by broad `pkill -f`, and check `pgrep -fa run_rover_landing` for a peer's
batch BEFORE launching (a peer's stack also caused my second attempt's port-8888 bind error).
