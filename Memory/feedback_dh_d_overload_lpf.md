---
name: feedback_dh_d_overload_lpf
description: dh_d overload root cause = 125Hz control / 42Hz image rate mismatch; fixed by 50ms LPF on ds_d (commit f4f91f2)
metadata: 
  node_type: memory
  type: feedback
  originSessionId: a377a083-d63b-447a-908e-12017cf609f1
---

> ⛔ **STAMP 2026-07-03: historical.** The TAU_DS dead-end + the K_rd=0/gamma_s=1.0 resolution were BOTH superseded by the combined-surface defaults (DHD-KF, DH_D_CAP, 787cf2d-era gains).
**Root cause of dh_d saturation (2026-06-09).**

Control loop runs at 125 Hz (dt=8ms); images arrive at **42 Hz** — only 37.5% of control steps have a new image frame. When a new frame does arrive, s_e_n jumps by ~0.015 normalized units, causing a step in ds_d of K_rp × Δs_e_n = 12 × 0.015 = 0.18 in one step → raw dh_d = 0.18/0.008 = **17 m/s³** median on axis-x. The `smooth4` window (4 samples, 34ms) halves this to ~9, but DH_D_MAX=5 was still clipping **70% of timesteps**.

This meant the c-term was being clipped to 5/17 = 29% of its true value at the median — severely distorting σ and κ adaptation. This is the reason clamp-relaxation (DH_D_MAX=50) was catastrophic: it revealed the underlying signal magnitude.

**Fix (DEAD-END — see n=5 result below):** first-order EMA on `ds_d` with τ = 50ms (`PLASMC_TAU_DS` env var, default now **0.0** — disabled):
```python
alpha = dt / (tau_ds + dt)
ds_d_lpf += alpha * (raw_ds_d - ds_d_lpf)
```
Simulation result: p50 17.8→3.5 m/s³, >50 clipping eliminated, >5 clipping 70%→40%. Adds ~50ms outer-loop lag.

**Why:** the 2/p_s Jacobian and G_s^{-1} back-map in SEN_FUNNEL cancel — gains K_rp/K_ri/K_rd are equivalent at small error in both legacy and SEN_FUNNEL modes. The problem was NOT coordinate mismatch but the ZOH effect of discrete image updates at 3× lower rate than the control loop.

**n=5 result (2026-06-09):** TAU_DS=0.05 median **6.411m WORSE than 3.80m baseline** (0/5 SP, 2 TL at 14.8/56.9m). LPF correctly reduces early dh_d clipping (0-2s: 0%, 2-4s: 13%), but adds 50ms outer-loop lag. Lateral flow underreport causes s_e_n drift from t≈2s; the 50ms lag delays outer PID correction → s_e_n_y reaches funnel breach 25% faster. Late-flight dh_d clipping (26-34%) is driven by large s_e_n×K_rp, not frame-rate mismatch — the LPF cannot help at that stage. **Default reverted to TAU_DS=0 (disabled).** Use PLASMC_TAU_DS>0 only as a diagnostic tool.

**How to apply:** PLASMC_TAU_DS=0 is the default (disabled). LPF mechanism exists and works (reduces dh_d spikes), but the lag cost exceeds the benefit under flow underreport conditions. If dh_d clipping is specifically suspected as causing σ distortion, re-enable via PLASMC_TAU_DS=0.02 (short lag) and verify at n≥5.
