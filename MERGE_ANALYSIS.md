# img_data.py Merge Plan: Ubuntu Gazebo → Pi Hardware

**Date:** 2026-07-08
**Source:** L:\Claude\Soft Landing\PX4_Gazebo\src\img_data.py (2949 lines)
**Target:** /home/doctor/ws/scripts/precise_landing/img_data.py (987 lines)

---

## Executive Summary

The Ubuntu/Gazebo version contains significant new algorithmic capabilities (EKF fusion, dense recovery, KF filtering) that are **camera-independent and should be ported to Pi hardware**. However, it also has Gazebo-specific infrastructure (GZ_Subscriber, Image_Node) and Gazebo-tuned calibration matrices that **MUST be replaced with Pi-specific values** without modification to the algorithms themselves.

---

## Part 1: Methods to Add (Camera-Independent, Port As-Is)

These methods are pure algorithmic implementations with NO camera intrinsics or Gazebo dependencies. They can be added to Pi img_data.py verbatim.

### A. Kalman Filter Suite (Low-lag filtering)
| Method | Lines | Purpose | Dependencies |
|--------|-------|---------|--------------|
| `_kf_step()` | 2215–2243 | Generic 2-state constant-velocity KF (value + rate) | numpy only |
| `_kf_update()` | 2245–2248 | Wrapper for corner-flow KF | `_kf_step()` |
| `_kf_feat_update()` | 2334–2363 | 4-channel 2-state KF for centroid (xc,yc,s,α) | numpy, self._kf_feat_* state vars |
| `_obs_vel_kf()` | 2364–2389 | Constant-velocity filter for centroid rate (ṡx,ṡy) | numpy |

**Status:** CAMERA-INDEPENDENT. These replace/augment legacy savgol filtering.

### B. EKF Fusion (Moving/stationary target discrimination)
| Method | Lines | Purpose | Dependencies |
|--------|-------|---------|--------------|
| `_ekf_fuse_step()` | 2250–2327 | Fuses corner + ring flow; estimates h_tr (target-relative) + h_tv (target velocity) + w | numpy, self._ekf_* state vars, self._H_corner/ring, self._R_corner/ring |

**Status:** CAMERA-INDEPENDENT. Requires initialization of `self._ekf_x`, `self._ekf_P`, `self._H_corner`, `self._H_ring`, `self._R_corner`, `self._R_ring` etc. (see Part 3).

### C. Dense Optical Flow Recovery (Marker dropout resilience)
| Method | Lines | Purpose | Dependencies |
|--------|-------|---------|--------------|
| `_dense_recover_anchor()` | 1116–1129 | Re-anchor dense-homography reference on clean 4-corner detection | cv2.calcOpticalFlowPyrLK, self._lk_params, self._dense_* state |
| `_dense_recover_step()` | 1131–1154 | Advance dense tracking per frame (survives partial dropout) | cv2.calcOpticalFlowPyrLK |
| `_dense_recover_quad()` | 1156–1183 | RANSAC homography fit to recover full quad from surviving points | cv2.findHomography, numpy |
| `_persist_extras()` | 1185–1232 | LK carry of extra corners through partial decodes; deduplication | cv2.calcOpticalFlowPyrLK, self._klt_persist_* params |

**Status:** CAMERA-INDEPENDENT. Pure CV operations. Requires `self._lk_params` initialization.

### D. Feature Point Generation (Dense sampling)
| Method | Lines | Purpose | Dependencies |
|--------|-------|---------|--------------|
| `_scaled_quad_points()` | 2161–2177 | Generate scaled quadrilateral point clouds (multi-scale density) | numpy |
| `_get_scaled_quadrilaterals()` | 2145–2150 | Helper to extract quads from marker corners | numpy |
| `_get_side_points()` | 2151–2160 | Extract equally-spaced points along quad sides | numpy |
| `_get_all_feature_points()` | 2178–2184 | Combine multi-scale points for lstsq input | numpy |
| `_fill_A()` | 2185–2213 | Build IBVS interaction matrix (2N×6 plate-form lstsq design) | numpy |

**Status:** CAMERA-INDEPENDENT. Purely geometric; no focal length or principal point used.

### E. Marker Geometry (Rotation, scale tracking)
| Method | Lines | Purpose | Dependencies |
|--------|-------|---------|--------------|
| `_marker_principal_angle()` | 2391–2428 | Extract 2π-periodic marker orientation (disambiguated via weighted moment) | numpy |
| `_marker_center_size()` | 2430–2436 | Static: extract center (xc,yc) and marker width from corners | numpy |
| `_update_sz_ratio()` | 2437–2461 | Track size ratio for nested markers (for priority selection) | numpy |
| `_fit_similarity()` | 2463–2484 | Static: RANSAC similarity-transform fit (2D+scale+rotation) | cv2.estimateAffinePartial2D |
| `_update_board_selfcal()` | 2488–2531 | Self-calibration of board geometry via marker corners | numpy |
| `_board_corners()` | 2532–2551 | Recover corner positions for a specific marker ID from board model | numpy |
| `_board_feature()` | 2552–2620 | Extract board feature vector (centroid s, scale, orientation α) | numpy, `_marker_principal_angle()` |

**Status:** CAMERA-INDEPENDENT. All purely geometric; no depth scaling.

### F. Filtering & Diagnostic
| Method | Lines | Purpose | Dependencies |
|--------|-------|---------|--------------|
| `_compute_savgol_output()` | 2869–2881 | Savitzky-Golay filter state snapshot (legacy alternative to KF) | scipy.signal.savgol_filter, deque |

**Status:** CAMERA-INDEPENDENT. Kept for A/B testing via IMG_FILTER=savgol.

---

## Part 2: Methods Requiring Camera Adaptation

These methods use camera intrinsics (`self.center`, `fx`, `fy`) or resolution. They are **algorithmically camera-independent but parameter-dependent**.

### Methods to Adapt (Preserve Algorithm, Replace Parameters)

| Method | Lines | Issue | Adaptation |
|--------|-------|-------|-----------|
| `_getVirtualPts()` | 2687–2739 | Uses `self.center` (cx,cy), `fx`, `fy` globals | Replace with Pi camera intrinsics. Algorithm is invariant; only px→normalized-ray conversion changes. |
| `_getRealPtsFromV()` | 2753–2768 | Inverse reprojection; uses `self.center`, `fx`, `fy` | Same: replace intrinsics, algorithm unchanged. |
| `_vframe_w()` | 2741–2750 | Rotates angular velocity to V-frame; uses gyro basis math | CAMERA-INDEPENDENT (no intrinsics). Port as-is. |
| `_compute_ring_flow()` | 1038–1115 | Ring radii are resolution-adaptive (`min(W,H)/2` fractions); reads from image resolution | **Preserve the adaptive-radius logic**, but re-tune the ring fraction constants for Pi resolution (currently 640×480 → adapt to Pi resolution). No depth mixing. |

### Adaptation Details

#### `_getVirtualPts()` and `_getRealPtsFromV()`
**Current (Gazebo):**
```python
cx, cy = self.center              # (240, 320) for 640×480 rotated
x = (pts[:, 0] - cx) / fx         # fx = 270 (Gazebo)
y = (pts[:, 1] - cy) / fy         # fy = 270 (Gazebo)
```

**For Pi Hardware:**
- Get Pi camera intrinsics (focal length, principal point) from calibration or hardware spec.
- Replace `fx`, `fy` module globals with `self.focal_x`, `self.focal_y` (instance vars, Pi-specific).
- Replace `self.center = np.array(self._resolution) / 2.0` logic: use Pi's actual calibrated principal point if available; otherwise center.
- The quaternion-based V-frame construction (`_getVirtualPts()` lines 2707–2739) is **completely camera-independent**; only the pixel→ray normalization changes.

#### `_compute_ring_flow()`
**Current (Gazebo, lines 1038–1115):**
- Ring radii are `frac * min(W, H) / 2` where `frac ∈ [0.17, 0.33, ..., 0.83]` (resolution-adaptive).
- Spans large area for noise reduction at 640×480.

**For Pi Hardware:**
- Query Pi resolution at init (e.g., via `imgstreamer` or `picamera2`).
- Recompute ring fractions for Pi resolution to maintain **same physical coverage** as Gazebo.
  - If Pi is 640×480: fractions unchanged.
  - If Pi is different: scale fractions proportionally (or re-tune offline on actual Pi data).
- The ring lstsq logic (`_fill_A()`) is identical; only the (x,y) positions of ring stations differ.

---

## Part 3: Gazebo Infrastructure to Replace

These sections **MUST be removed or adapted**; they are Gazebo-specific.

### A. Image Acquisition (`__init__`, `_wait_for_images()`)

**Gazebo code (lines 59–60):**
```python
self._image_node = Image_Node(time_keeper=time_keeper, controller=controller)
self._image_subscriber = GZ_Subscriber(self._image_node)
```

**For Pi Hardware:**
- Remove `GZ_Subscriber` and `Image_Node` imports.
- Replace with Pi's image acquisition (e.g., `imgstreamer` reader, `picamera2`, or whatever the current Pi codebase uses).
- Ensure `_wait_for_images()` (line 1024) is adapted to read from Pi stream instead of `self._image_subscriber`.

**Minimal Interface Change:**
```python
# Pi version should maintain:
def _wait_for_images(self):
    # ... Pi-specific read from imgstreamer or picamera
    imgs, quats = ...  # get images + quaternions from Pi pipeline
    return imgs, quats, any_timestamps
```

### B. Init: Resolution & Camera Intrinsics (lines 67–85)

**Gazebo code:**
```python
self._resolution = self._image_node.getImgResolution()  # (480, 640) from ROS msg
self.center = np.array(self._resolution) / 2.0
```

**For Pi Hardware:**
- Query Pi resolution from imgstreamer metadata or camera config.
- Compute center accordingly.
- Update `fx`/`fy` to Pi's focal length (currently hard-coded at module level, line 32–33).

---

## Part 4: Sensor Calibration Matrices (CRITICAL: Do NOT Overwrite Pi Values)

The Ubuntu version has Gazebo-tuned matrices. **The Pi version MUST retain its own calibration matrices**.

### Current Gazebo Values (Lines 130–171)

```python
self._sensor_cal_hw = np.array([
    [+1.4272, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000],   # h_x = observer beta_x
    [+0.0000, +1.0253, +0.0000, +0.0000, +0.0000, +0.0000],   # h_y
    [+0.0535, -0.0044, +0.4973, +0.0000, +0.0000, +0.0000],   # loom row
    ... (all-zero rows for wx,wy)
    [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.8439]])  # w_z

self._sensor_cal_s = np.diag([1.1391, 1.1437, 1.0, 1.0])      # observer/1m-nested centroid

self._sensor_cal_ring = np.array([
    [+0.6523, +0.0317, +0.1372, -0.0371, +0.6014, -0.1252],
    ... (6×6 matrix)
])
```

### Pi Hardware Values (From Task Description)

According to CLAUDE.md memory and task description:
```python
_sensor_cal_hw  = np.diag([1/6, 1/6, 1/6, 1, 1, 1])      # legacy Pi calibration
_sensor_cal_s   = np.diag([1/12, 1/12, 1, 1])           # legacy Pi calibration
_sensor_cal_ring = np.eye(6)                            # legacy Pi ring cal
```

### Merge Strategy

**Option A (Recommended): Preserve Pi Defaults, Flag for Recalibration**

```python
# At init, prefer Pi calibration; fallback to Gazebo only if env var set
self._sensor_cal_hw = np.diag([1/6, 1/6, 1/6, 1, 1, 1])  # Default Pi
if os.environ.get("USE_GAZEBO_CAL", "0") == "1":
    self._sensor_cal_hw = np.array([...Gazebo matrix...])

self._sensor_cal_s = np.diag([1/12, 1/12, 1, 1])         # Default Pi
self._sensor_cal_ring = np.eye(6)                        # Default Pi
```

**Option B (Better): Load from Environment or Config File**

```python
# Allow per-deployment calibration without code changes
if os.environ.get("CAL_FILE"):
    # Load from .npz or JSON
    cals = np.load(os.environ["CAL_FILE"])
    self._sensor_cal_hw = cals['sensor_cal_hw']
    self._sensor_cal_s = cals['sensor_cal_s']
    self._sensor_cal_ring = cals['sensor_cal_ring']
else:
    # Hardcoded defaults per deployment
    if platform == "pi":
        self._sensor_cal_hw = np.diag([1/6, 1/6, 1/6, 1, 1, 1])
    elif platform == "gazebo":
        self._sensor_cal_hw = np.array([...Gazebo matrix...])
```

---

## Part 5: Initialization of EKF/KF State Variables

The new Kalman filter methods require initialization of state variables in `__init__()`. Gazebo version has these; Pi version must add them.

### Required State Variables (to add to Pi's `__init__()`)

```python
# Corner flow KF (6-state × 2-channel: value + rate)
self._kf_x = np.zeros((6, 2))            # [h; w] values and rates
self._kf_P = np.eye(6) * 1.0             # covariance
self._kf_initialized = False
self._kf_prev_t = None
self._kf_q = 5.0                         # process noise (env-tunable)
self._kf_r = 0.1                         # measurement noise

# Ring flow KF (separate state)
self._kf_x_ring = np.zeros((6, 2))
self._kf_P_ring = np.eye(6) * 1.0
self._kf_initialized_ring = False
self._kf_prev_t_ring = None

# EKF fusion state (9-state: h_tr + h_tv + w)
self._ekf_x = np.zeros(9)                # [h_tr(3), h_tv(3), w(3)]
self._ekf_P = np.eye(9)
self._ekf_init = False
self._ekf_prev_t = None
self._ekf_Q = np.eye(9) * 0.1            # process noise (random walk)

# EKF measurement matrices
self._H_corner = np.block([[np.eye(3), np.zeros((3,3)), np.zeros((3,3))],
                           [np.zeros((3,3)), np.zeros((3,3)), np.eye(3)]])
self._H_ring = np.eye(9)
self._H_ring_loom = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 1]])  # w_z only
self._H_htv_z = np.array([[0, 0, 0, 0, 0, 1, 0, 0, 0]])      # h_tv[2] only

self._R_corner = np.diag([...])          # measurement covariance (corner)
self._R_ring = np.diag([...])            # measurement covariance (ring)
self._R_ring_loom = np.array([[...]])    # measurement covariance (ring loom scalar)
self._R_htv_z = np.array([[...]])        # measurement covariance (h_tv[2] prior)

# EKF tuning
self._ring_loom_thresh = 4               # n_corn threshold for ring loom takeover
self._htv_z_prior_on = True              # ground-target h_tv[2]→0 prior
self._loom_sign_guard = True             # clamp loom ≤ 0
self._loom_decouple = False              # raw loom bypass for cal experiments
self._fuse_ring = True                   # EKF fusion on/off (env-tunable)
self._loom_stale = 0; self._loom_stale_max = 5; self._loom_decay = 0.9

# Centroid KF (4-channel feature: xc, yc, s, alpha)
self._kf_feat_x = np.zeros((4, 2))
self._kf_feat_P = np.tile(np.eye(2), (4, 1, 1))
self._kf_feat_initialized = False
self._kf_feat_prev_t = None
self._kf_feat_q = 1.0                    # tuned separately from flow KF
self._kf_feat_r = 0.5
self._kf_feat_last_n = 0                 # track update count for lazy stepping

# Observer velocity KF (centroid rate smoothing)
self._obs_kf_x = None; self._obs_kf_y = None
self._obs_kf_Px = None; self._obs_kf_Py = None
self._obs_kf_t = None
self._obs_kf_q = 10.0                    # higher q for faster tracking
self._obs_kf_r = 0.2

# Dense recovery state
self._dense_recover = True               # on/off flag
self._dense_recover_active = False       # currently tracking
self._dense_canon_quad = None            # canonical corner positions
self._dense_canon_pts = None             # scaled quad points (dense sampling)
self._dense_track_pts = None             # tracked positions of canon_pts
self._dense_ref_img = None               # reference image for LK
self._dense_frames_since_anchor = 0
self._dense_pts_per_side = 15            # points per quad side (total ~180)
self._dense_recover_min_pts = 30         # min inliers for RANSAC fit
self._dense_recover_max_frames = 20      # max staleness before abandon
self._dense_recover_min_inlier_frac = 0.5  # min inlier ratio
self._dense_recover_ransac_px = 2.0      # RANSAC pixel threshold

# Ring-station tracking
self._ring_stations = None               # (N, 2) concentric-ring positions
self._ring_log_on = True                 # log ring flow
```

**Note:** Most of these have environment-variable overrides (e.g., `self._kf_q = float(os.environ.get("IMG_KF_Q", "5.0"))`). Check Gazebo init (~lines 250–900) for all tuning constants.

---

## Part 6: Detailed Merge Checklist

### Phase 1: Add Camera-Independent Methods (No Risk)

- [ ] Copy `_kf_step()`, `_kf_update()`, `_kf_feat_update()`, `_obs_vel_kf()`
- [ ] Copy `_ekf_fuse_step()`
- [ ] Copy dense recovery: `_dense_recover_anchor()`, `_dense_recover_step()`, `_dense_recover_quad()`, `_persist_extras()`
- [ ] Copy feature point generation: `_scaled_quad_points()`, `_get_*()`, `_fill_A()`
- [ ] Copy marker geometry: `_marker_principal_angle()`, `_marker_center_size()`, `_update_sz_ratio()`, `_fit_similarity()`, `_update_board_selfcal()`, `_board_corners()`, `_board_feature()`
- [ ] Copy `_compute_savgol_output()`

### Phase 2: Adapt Camera-Dependent Methods (With Testing)

- [ ] Port `_getVirtualPts()` with Pi intrinsics
- [ ] Port `_getRealPtsFromV()` with Pi intrinsics
- [ ] Port `_vframe_w()` (copy as-is; it's camera-independent)
- [ ] Adapt `_compute_ring_flow()` for Pi resolution
- [ ] Add `self.focal_x`, `self.focal_y` to init (currently module-level globals `fx`, `fy`)

### Phase 3: Replace Gazebo Infrastructure

- [ ] Remove `from gz_subscriber import GZ_Subscriber, Image_Node`
- [ ] Replace `__init__()` lines 59–60 with Pi image acquisition setup
- [ ] Adapt `_wait_for_images()` to read from Pi pipeline

### Phase 4: Preserve Hardware Calibration

- [ ] Replace Gazebo `_sensor_cal_hw`, `_sensor_cal_s`, `_sensor_cal_ring` with Pi values (Option A or B above)
- [ ] Add env-var override for recalibration workflows

### Phase 5: Add KF/EKF State Initialization

- [ ] Add all state variables listed in Part 5 to Pi's `__init__()`
- [ ] Set default process/measurement noise covariances per Pi tuning (or env-vars)

### Phase 6: Integration & A/B Testing

- [ ] Run `IMG_FILTER=kf` vs `IMG_FILTER=savgol` A/B (KF should be ~2× lower lag)
- [ ] Run `FLOW_FUSE_RING=1` vs `FLOW_FUSE_RING=0` A/B (fusion should handle dropout better)
- [ ] Enable `DENSE_RECOVER=1` and test dropout resilience

---

## Part 7: Critical Gotchas & Dependencies

1. **Kalman filter state is NOT reset frame-to-frame.** Stale state across a marker loss will bias the next detection. The code handles this via marker-switch detection in `_imgProcess()` (lines 1306–1313 in Gazebo); ensure Pi version includes this reset logic.

2. **EKF h_tv[2] prior (ground-target assumption).** The EKF assumes targets don't change altitude. For rovers/moving targets, keep `_htv_z_prior_on=True` to project the ego-loom into h_tr[2]. Disable only for flying targets.

3. **Ring loom decay (_loom_stale, _loom_decay).** If corner flow disappears, the EKF's h_tr[2] (loom) is not updated for `_loom_stale_max` frames; then it decays toward 0 to prevent a frozen stale value from commanding ascent. This is defensive; if Pi's ring flow is noisy, tune `_loom_stale_max` and `_loom_decay`.

4. **Dense recovery anchoring.** Dense recovery re-anchors ONLY on a clean ArUco decode (all 4 corners, high confidence). If Pi's ArUco detection is marginal, the dense pool may never anchor → it won't survive dropouts. Test with `DENSE_RECOVER=1` and verify it anchors on every successful decode.

5. **Ring radii are resolution-adaptive.** If Pi uses a different resolution than 640×480, the ring station positions must be recomputed. The Gazebo code does this automatically (lines ~250–260); ensure Pi's `_compute_ring_flow()` does the same.

6. **LK parameters are Gazebo-tuned for 640×480.** Pi's resolution may differ; `_lk_params` (winSize, maxLevel, etc.) may need re-tuning. Env-overridable via `FLOW_LK_WIN`, `FLOW_LK_LVL`, etc. (lines 215–219).

7. **Savgol filter is legacy.** The new KF is the default (IMG_FILTER=kf). The legacy savgol (IMG_FILTER=savgol) is kept for A/B comparison but adds ~110 ms group delay. Do NOT use savgol in production on Pi.

8. **Sensor calibration is frame-invariant.** If Pi's focal length or principal point differ from Gazebo, the calibration matrices will be OFF. The task description notes that Pi has legacy cal values; these should remain unless recalibrated on actual Pi hardware.

---

## Part 8: Recommended Merge Order

1. **Backup Pi img_data.py** → `img_data_pi_v1_backup.py`
2. **Add all camera-independent methods** (no functional change expected)
3. **Add KF/EKF state initialization** (initialize to zero, not used yet)
4. **Port camera-dependent methods** (with Pi intrinsics substituted)
5. **Replace GZ_Subscriber with Pi acquisition** (verify images flow)
6. **Replace calibration matrices** (preserve Pi defaults)
7. **Enable one filter at a time** (IMG_FILTER=kf first, test lag reduction)
8. **Enable EKF fusion** (FLOW_FUSE_RING=1, test dropout handling)
9. **Enable dense recovery** (DENSE_RECOVER=1, test dropout resilience)
10. **Full regression test** (multi-IC landing suite, A/B KF vs savgol)

---

## Summary Table: Methods by Port Status

| Category | Status | Methods | Lines |
|----------|--------|---------|-------|
| **Port As-Is** | ✅ | KF suite, EKF, dense recovery, feature generation, marker geometry, savgol | 1116–1183, 2161–2620, 2687–2881 |
| **Adapt (Keep Algorithm)** | ⚠️ | `_getVirtualPts()`, `_getRealPtsFromV()`, `_compute_ring_flow()` | 2687–2768, 1038–1115 |
| **Camera Intrinsics** | 🔧 | fx, fy, center, resolution | Module 32–33, init 67–85 |
| **Gazebo Infrastructure** | ❌ | GZ_Subscriber, Image_Node, `_wait_for_images()` | 24, 59–60, 1024 |
| **Calibration Matrices** | 🔒 | `_sensor_cal_hw`, `_sensor_cal_s`, `_sensor_cal_ring` | 130–171 (PRESERVE PI VALUES) |
| **State Initialization** | ➕ | KF/EKF state vars | (add to Pi init) |

