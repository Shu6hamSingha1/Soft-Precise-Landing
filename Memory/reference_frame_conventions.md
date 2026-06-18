---
name: frame-conventions
description: PX4 NED/FRD vs Gazebo ENU/FLU — verified conversion matrices for analysis notebooks
metadata: 
  node_type: memory
  type: reference
  originSessionId: 367ac3aa-13f8-41ea-b23e-c929f5e1e50d
---

## Frame conventions in this project (verified empirically)

| Source | World frame | Body frame |
|---|---|---|
| PX4 (telemetry) | NED (x=North, y=East, z=Down) | FRD (x=Forward, y=Right, z=Down) |
| Gazebo (GT) | ENU (x=East, y=North, z=Up) | FLU (x=Forward, y=Left, z=Up) |

## MAVSDK telemetry — what frame each field is in

- `tel['Position Body']` (`x_m, y_m, z_m`): **NED-at-takeoff** (world-fixed, NOT body-rotating despite the type name). Verified by `d/dt(Position Body) ≈ Velocity Body` per-frame across input-cal data.
- `tel['Velocity Body']` (`x_m_s, y_m_s, z_m_s`): same NED-at-takeoff frame as Position Body. So `inv(R_PX4) @ velocity` gives body-FRD-current velocity.
- `tel['Quaternion']`: body→NED orientation (PX4 standard). `ahrs.Quaternion([w,x,y,z]).to_DCM()` returns **R_world←body** ↔ `v_world = R @ v_body`. Equivalently the 3rd column of R is the body-z-axis expressed in world coords, so `R @ [0,0,1]_world = world-down-in-body` only if interpreted in PX4's NED convention. Verified by pitched-30° test.
- `tel['Angular Velocity Body']` (`roll_rad_s, pitch_rad_s, yaw_rad_s`): body-FRD rates from the gyro.
- `tel['Acceleration']`: body-FRD specific force (includes gravity).

## Conversion matrices (used in plotter notebooks)

```python
from ahrs import DCM
FRD_2_FLU    = np.array(DCM(x=180))                       # body-side: FRD ↔ FLU
NED_from_ENU = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])  # world-side: ENU → NED (self-inverse)
```

`NED_from_ENU` is self-inverse (also converts NED → ENU). `FRD_2_FLU` is also self-inverse (DCM(x=180) is its own inverse).

## Two equivalent strategies for body-frame analysis

**Strategy A — common body-FRD-current frame (matches `~/ws/.../plotter_landing_test.ipynb`):**
- TEL: `inv(q_PX4.to_DCM()) @ NED_pos` → body-FRD
- GT: `inv(q_gazebo.to_DCM() @ FRD_2_FLU) @ ENU_pos` → body-FRD

Both end in the same body-FRD-current frame. World-frame mismatch (NED vs ENU) cancels out because each side's rotation handles its own world convention.

**Strategy B — explicit NED-from-ENU on GT (used in current `plotter_output_calibration.ipynb`):**
- TEL: `pos_NED − pos_NED[0]` (no rotation, already in NED-at-takeoff)
- GT: `(NED_from_ENU @ (W_pos − W_pos_spawn).T).T` (ENU → NED via world swap+flip)

Both end in NED-at-takeoff. Works only when spawn yaw is 0; non-zero spawn yaw requires additional R_z correction.

A is more robust and equivalent for static-yaw drones. B is simpler when spawn yaw is consistently 0 (Gazebo default for the aruco world).

## Euler angle extraction across frames

For `R_NED_from_FRD` (PX4-like), Tait-Bryan ZYX:
```python
roll  = arctan2(R[2, 1], R[2, 2])
pitch = arcsin(-R[2, 0])
yaw   = arctan2(R[1, 0], R[0, 0])
```

Element-wise Euler-vector flips between conventions (e.g. `[1, -1, -1]` for FLU→FRD) are correct only for small-angle rotations near identity. For arbitrary spawn yaw + body convention, do the conversion at the rotation-matrix level then re-extract Euler.

## Quaternion delta-from-spawn (body-frame relative rotation)

```python
def q_mul(a, b):  # [w,x,y,z] Hamilton product
    w1,x1,y1,z1 = a; w2,x2,y2,z2 = b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2,
                     w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2,
                     w1*z2+x1*y2-y1*x2+z1*w2])
def q_conj(q): return np.array([q[0], -q[1], -q[2], -q[3]])

# Body-frame delta: spawn^-1 * current  (NOT current * spawn^-1, which gives world-frame delta)
q_delta_body = q_mul(q_conj(q_spawn), q_now)
```

Method choice verified: synthetic test with spawn yaw=+96°, +10° roll commanded → body-frame method gives (10°, 0°, 0°); world-frame method gives wrong components.
