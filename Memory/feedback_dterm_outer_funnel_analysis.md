---
name: feedback_dterm_outer_funnel_analysis
description: "D-term (K_rd) root cause analysis in SEN_FUNNEL outer PID: dzeta_s spike mechanism, outer funnel p_s/p_10 geometry, gamma_s too slow — binding constraint 2026-06-09"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: a377a083-d63b-447a-908e-12017cf609f1
---

**D-term spike mechanism (2026-06-09, fully decomposed).**

Peak ds_d=-29 rad/s breakdown at corner instability event (t=5.15s, rep3 K_rp=9 batch):
- P-term: +1.01 rad/s (r=−0.15, barely off-center)
- I-term: +0.02 rad/s (clamped)
- **D-term: −28.70 rad/s (99.7% of spike)**

Mechanism: close-range ArUco corners jump → s_e_n swings from r=−0.72 to r=−0.15 in one 42Hz frame → `dzeta_s = 54 rad/s²` → `K_rd × G_s_inv × dzeta_s = 1.4375 × 0.37 × 54 = 28.7 rad/s`.

`smooth4` (4 samples = 95ms at 42Hz) already applied; the spike is the jump itself, not HF noise.

**K_rd=0 n=5 result:** median 11.96m (3 TL), vs K_rp=9 baseline 3.06m. Removing D-term eliminates ds_d spikes (max 3-5 rad/s vs 29) but exposes the **underlying PI drift issue**: s_e_n grows to 2.5-6.4 (funnel breach) because outer PI is too weak to arrest drift.

**K_rd=0 + gamma_s=1.0 (BAKED DEFAULT as of 2026-06-10):** gamma_s=1.0 sweep winner — fastest outer funnel contraction. xy_med=1.32m, κ_max_med=1.09 (lowest of sweep). ⚠️ the logged "1/5 SP at 0.03m" is **UNVERIFIED** — no saved recording; the only sub-10cm rep in all saved R3 data is a frozen-GT artifact (see [[feedback_false_sp_frozen_gt]]). P-term now arrests drift before altitude amplification dominates. **BAKED: K_rd=0.0, gamma_s=1.0 in controller.py defaults.** Known issue: descent bootstrap at 6m (see [[feedback_descent_softness]]) — next to fix.

**K_rd was load-bearing** for drift arrest between corner instability events — it provides corrective torque against dzeta_s sign changes. gamma_s=1.0 replaces this role via fast funnel pressure. Both diagnoses were correct; gamma_s=1.0 resolves the root cause (funnel authority) rather than patching the symptom (spike damping).

---

**Outer funnel geometry (p_s, p_10, gamma_s).**

```python
s_e_n = s_e[:2] / p_10           # dimensionless, ±1 = image edge
p_10  = center / focal            # [240,320]/270 = [0.889, 1.185] rad (half-FoV per axis)
r     = s_e_n / p_s(t)            # funnel coordinate, must stay |r| < 1
zeta_s = log((1+r)/(1-r))         # PPC transform; PID runs on zeta_s
```

`p_s(t) = (p_s_0 − p_s_inf)·exp(−gamma_s·t) + p_s_inf`
Defaults: p_s_0=1.2, p_s_inf=0.1, gamma_s=0.1

| t | p_s | Allowed |s_e_n| | r at s_e_n=0.1 |
|---|-----|---------|---------|
| 0s | 1.200 | 1.067 rad | 0.083 |
| 4s | 0.837 | 0.744 rad | 0.119 |
| 8s | 0.591 | 0.525 rad | 0.169 |
| ∞  | 0.100 | 0.089 rad | 1.000 |

**Key insight:** p_s_0=1.2 is barely wider than the x-axis FoV edge (p_10_x=1.185). At gamma_s=0.1, p_s(4s)=0.837 → at s_e_n=0.1 the ratio r=0.119, zeta_s=0.239 → P-term = K_rp × G_inv × zeta_s ≈ 9 × 0.84 × 0.239 ≈ **1.8 rad/s**. Very weak against lateral drift that compounds via 1/alt amplification.

**Binding constraint:** outer funnel contracts so slowly that the P-term provides negligible drive until very late, when altitude is already low and 1/alt has amplified a small xy offset into a large normalized image error. By then the funnel is breached.

**Fix lever: gamma_s** — faster contraction makes r grow sooner for the same s_e_n, zeta_s grows, P-term drive increases while there is still altitude headroom to correct. gamma_s sweep (0.2/0.3/0.5/1.0, K_rd=0) running 2026-06-09.

**Why:** `p_s_inf=0.1` corresponds to |s_e_n|<0.089–0.119 rad at touchdown — well within ArUco detection range. The terminal target is fine; only the contraction rate is too slow.

**How to apply:** When outer loop (image error) fails to arrest drift despite reasonable K_rp, check gamma_s first. The funnel must be tight enough to give the P-term authority early. Env vars: `PLASMC_XIS_X`, `PLASMC_XIS_Y`.
