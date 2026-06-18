---
name: outputcal-flow-validation-vframe
description: "output-cal optical-flow equation validation (plotter_output_calibration.ipynb cell 40) is now done in the gravity-leveled VIRTUAL frame (runtime-faithful), per user decision 2026-06-04 — NOT body-FRD. Corrects the R_V_from_body=I assumption. img_data solves [h;w] in V frame; body-frame check is cleaner but V-frame is what the runtime actually does."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 75fad30b-d46f-4f5a-8783-a865d1b91031
---

**FRAME MODEL (corrected 2026-06-04, user-confirmed against the canonical reference notebooks):**
- **Camera frame = body-FRD** — axes aligned, NO rotation between them (SDF mount + cv2 rotation;
  origin may be offset). Empirically verified: image `s_x`↔body-forward r=+0.89, `s_y`↔body-right r=+0.95.

> **⚠️ CORRECTION (2026-06-06): the "Empirical" conclusion below (V-frame residual
> 0.76 = EKF-leveling noise; body-frame cleaner) is FALSE.** The V-frame check's
> higher residual was the **cell-20/40 RHS using the FULL body rotation `V_w_tug`
> (roll/pitch included) instead of yaw-only.** The V-frame correctly *removes*
> roll/pitch, so `ṡ_V` is ~0.66× the raw flow — that is CORRECT, not noise; the
> RHS over-predicted by the phantom roll/pitch term. **Fix `86509bb`:** zero
> `V_w_tug[:, :2]`. After the fix the V-frame check is the GOOD one — corr
> 0.81→**0.94**, slope→0.96 — *better* than the body-frame check. So "body-frame
> cleaner / V-frame honest-but-noisy" is **superseded**, and the "noisy/lagged
> EKF quaternion injects flow noise" attribution was wrong (forward-predicting
> the attitude does nothing; the leveling is correct). The frame-model + naming
> content elsewhere in this note stays valid. See [[feedback_vframe_rhs_yaw_only]].
- **Virtual (V) frame = gravity-LEVELED camera frame = `rotz(yaw)`** (roll/pitch removed, yaw kept).
  So `R_{V←body}` is the roll/pitch LEVELING rotation — **identity ONLY when level, ≠ I under tilt.**
- **`R_V_from_body = I` is WRONG** (it conflates camera=body, true, with V=body, false). That claim was
  added 2026-06-01 in commit `fce9b84` ("Calibration pipeline…"); corrected 2026-06-04 in output-cal
  cells 5/6/13/39/40, input-cal cells 0/24, and the `img_data.py` `_getVirtualPts` comment (~L1110).
- **Canonical references** (`~/ws/scripts/soft_precise_landing/`): `plotter_virtual_calibration.ipynb`
  builds `W_R_Bz = DCM(z=yaw) @ FRD_2_FLU`, carries `B_x_tu`/`V_x_tu`, `B_y_g`/`V_y_g` distinct, each
  with its OWN depth (`/B_x_tu[2]` vs `/V_x_tu[2]`). `plotter_input_calibration.ipynb` has NO virtual
  frame — angular velocity is pure body-FRD (input-cal is rate-loop tracking; body-FRD is correct there).
- **img_data `_getVirtualPts` builds V via cross-product** (`x = body_y × gravity`), which differs from
  the canonical `rotz(yaw)` by a 2nd-order `roll·pitch` term (≤1° at 11° tilt, corr 1.0 with |roll·pitch|;
  changes validation rel-err <1e-4). User decision: keep the cell-40 RHS on the **runtime cross-product
  frame** (matches the logged LHS exactly) — not `rotz(yaw)`.

**User decision (2026-06-04): validate the IBVS optical-flow equation `ṡ = L·[h;w]` in the
gravity-leveled VIRTUAL (V) frame, runtime-faithful — even though it scores worse than the
body-frame check — because that is the frame `img_data.py` actually computes flow in.
`[h;w]` on the RHS are GT VIRTUAL image velocity + angular velocity (`B_y_g_V`, `B_w_tug_V`), both
rotated into V (NOT body-FRD). `h_V` is unambiguous; `w_V = V_R_body @ B_w_tug` carries a small
2nd-order subtlety (rate vs the leveled virtual camera) that folds into the EKF-leveling residual.**

**Why the frame matters (corrects the old `R_V_from_body = I` belief in [[imgdata-gt-clock-skew]]
and the input-cal notebook):** `R_V_from_body = I` holds only when level. In practice the drone
tilts (multisine/yaw phases), so V ≠ body: image-normalized vs virtual ArUco corners correlate
only ~0.94, and the V-leveling pulls the marker toward center (e.g. image ~0.18 vs virtual ~0.04).

**What img_data does:** `_imgProcess` builds `A = _fill_A(V_flow_norm[1])` and
`Y = (V_flow_norm[1]-V_flow_norm[0])·fps` from gravity-leveled VIRTUAL points
(`_getVirtualPts`), then `lstsq(A,Y)` → `[h;w]` is solved IN THE V FRAME. So the validation must
use V-frame quantities on both sides.

**Empirical (multisine recording, prototyped before editing):**
- Body-frame check: rel-err 0.41, model corr 0.955.
- V-frame check (runtime-faithful): rel-err 0.76, model corr 0.869.
- The gap is NOT a model/transform error: at low tilt the PREDICTED sides agree corr **0.999**
  (my h_V/w_V transform is correct); the degradation is entirely on the MEASURED side — the
  leveled virtual-point flow is ~0.66× the raw image flow, corr 0.90. Cause: the runtime levels
  each frame with the live (noisy, lagged) EKF quaternion, injecting flow noise the smooth
  GT-derived `[h;w]` cannot reproduce. So body-frame is the cleaner *physics* check; V-frame is
  the honest *runtime* check. Per-axis `h`/`w` cal (cells 12/14) is the complementary view.

**Edits made (plotter_output_calibration.ipynb, uncommitted):**
- **cell 6**: replaced `R_V_from_body = np.eye(3)` / `B_y_g_V = B_y_g` aliases with a per-sample
  V-frame computation. V basis = same as `xc_gt`/`yc_gt` (z = world-down NED, x = UAV yaw dir).
  `h_V = (V_R_NED @ W_v_tu) / z_V` with `z_V = W_x_tu[2]` (altitude = depth along V optical axis,
  NOT body-z); `w_V = V_R_NED @ (R @ B_w_tug)`. `R_V_from_body = None` (no longer constant).
  Body-FRD `B_y_g`/`B_w_tug` kept for cells 12/14.
- **cell 40**: LHS = `(V_new - V_old)·fps` from `Virtual Feature Pts` (already normalized, no
  center/focal); L built from virtual corners; RHS = `L·[B_y_g_V; B_w_tug_V]`.
- **cell 39**: markdown rewritten to describe the V-frame validation + the expected higher-residual caveat.

**NAMING CONVENTION (enforced 2026-06-04, commit ed27641): frame prefix = ACTUAL frame.**
`B_` = body-FRD, `V_` = virtual (gravity-leveled camera). Old image-velocity notation `y`→`h`.
Output-cal + 7 cal tools renamed: `B_y_g`→`B_h_g` (body), `B_y_g_V`→`V_h_g`, `xc_gt/yc_gt`→
`V_xc_g/V_yc_g`, `y_cal`→`V_h_cal`, `w_cal`→`V_w_cal`, `xc_cal/yc_cal`→`V_xc_cal/V_yc_cal`,
`B_w_tug_V`→`V_w_tug`. Body kept: `B_x_tu`, `B_v_tu`, `B_w_ug/tg/tug`, `B_h_g`.

**Discovered during rename — `B_y_g` was FRAME-INCONSISTENT across tools** (same name, different
frame): VIRTUAL (V_*_NED projection) in `aggregate_calibration_phased` / `derive_board_cal` /
`find_camera_rotation` → now `V_h_g`; BODY (`inv(R)@v`) in `validate_pose_transforms` /
`analyze_calibration` / `aggregate_calibration` / `tune_savgol` → now `B_h_g`.

**OPEN SEMANTIC FLAG (naming now exposes it; NOT yet fixed — would change cal values):** the cal
derivation mixes frames per channel. img_data outputs V-frame, but several GT comparisons use BODY:
the legacy/diagnostic tools (analyze_calibration, aggregate_calibration, tune_savgol) and notebook
cells 12/14 compare BODY GT velocity (`B_h_g`) against V-frame calibrated output; even the canonical
`aggregate_calibration_phased` uses `V_h_g` (virtual) for h but `B_w_ug` (body) for the w-axis cal.
At low tilt body≈virtual so the derived cal is ~right, but for exactness the GT should be V-frame on
all 6 channels. Decide separately (behavior change + re-validation).

See [[getvirtualpts-g-sign]] (V-frame leveling), [[imgdata-gt-clock-skew]] (R_V=I claim corrected).
