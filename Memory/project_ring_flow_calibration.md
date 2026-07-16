---
name: ring-flow-calibration
description: "Ring (texture-free) optical-flow calibration: pipeline + an APPLIED M_ring (2026-06-06, transfer mode: ring calibrated against the calibrated corner over same-clock recordings). derive_ring_cal.py has RING_CAL_MODE=transfer (default)|gt. _sensor_cal_ring now non-identity + getRingFlowAngVel applies it. Notebook + validate_output_flow AUTO-LOAD cal from img_data.py."
metadata:
  node_type: memory
  type: project
  originSessionId: ring-cal-2026-06-06
---

The ring lstsq is NOT depth-mixed (see [[ring-depth-mixing-falsified]]), so it gets a
fixed 6x6 `M_ring` derived exactly like the corner cal. Pipeline (all committed-ready,
2026-06-06):

- **img_data.py**: `getRawRingFlowAngVel()` (raw ring `[h;w]`, pre-cal) mirrors
  `getRawOptFlowAngVel`. `_sensor_cal_ring` (a 6x6, **identity placeholder**) + calibrated
  getter `getRingFlowAngVel()` = `_sensor_cal_ring @ ring-KF`. Runtime applies NO cal to the
  ring today (logged raw), so identity == current behaviour; ring is a SAFETY NET (control
  consumes corner flow), so identity is inert for landing.
- **apps/record_output_calibration.py**: now co-samples `getRawRingFlowAngVel()` into the GT dict
  as `"Ring Opt Flow Ang Vel"` (alongside the corner raw), so future phased recordings
  carry the ring co-sampled with GT.
- **tools/derive_ring_cal.py**: fits `GT[h;w] = M_ring @ ring_raw`, phase-segmented, same
  recipe as derive_board_cal. Auto-detects **co-sampled** ring (preferred) vs **retro-aligns**
  the ring from Img_Data via the shared corner signal (existing runs). Env
  `RING_CAL_MODE` = **transfer** (default) | gt. `RING_CAL_COSAMPLED_ONLY` (gt mode) =
  auto / 1 / 0. `RING_CAL_FULL_W=1` keeps full-w GT (gt mode).

**STATUS: M_ring DERIVED + APPLIED (2026-06-06, mode=transfer).** Key insight: the GT-direct
path on existing data is limited by Img/GT **cross-clock alignment** (locks the yaw phase at
r=1.00 but smears x/y/z → retro M_ring noisy, R^2 0.28–0.70, Wz gain 4.4). The **transfer**
mode fixes this: calibrate the ring against the ALREADY-CALIBRATED CORNER as a transfer
standard (`M_ring@ring ≈ M_corner@corner`), both raws SAME-CLOCK in Img_Data, over 13 rich
recordings (RingFlow landings + output runs). Result R^2 **Hx 0.63 Hy 0.84 Hz 0.76 Wz 0.89**
(approaching corner); Wx/Wy rows exactly 0 (inherited from corner standard); makes
ring_cal≈corner_cal → continuous marker→ring handoff. **CAVEAT:** Wz gain 3.45 with high
inter-run STD — ring sees yaw weakly (concentric, low texture); trust h-block, ring-yaw coarse.
**PROVISIONAL:** keyed to the current corner M (re-derive if it changes). The GT-DIRECT cal
(definitive) still wants a co-sampled re-record: re-run output_calibration (now co-samples the
ring) then `RING_CAL_MODE=gt` (auto co-sampled-only).

**Single source of truth for the CORNER cal:** the cell-4 of
`plotter_output_calibration.ipynb` and `tools/validate_output_flow.py` now
**auto-load `_sensor_cal_hw`/`_sensor_cal_ring` from src/img_data.py** (regex-parse the
np.array literal) instead of hard-coding — so iterating the cal in img_data.py no longer
needs a manual re-sync. validate_output_flow's earlier ring bug (applied the CORNER
CAL to the ring, which the runtime never does) is fixed: ring now uses `_sensor_cal_ring`.

See [[cal-data-provenance-cleanup]], [[reference_aggregate_calibration]],
[[feedback_vframe_rhs_yaw_only]].
