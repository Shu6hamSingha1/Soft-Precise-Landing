# Ubuntu Handoff — resume CBF / SEN_FUNNEL work (2026-06-14)

Resumes the work done in the Windows/MATLAB session.

> **PRIORITY: the major focus is resolving the IC5 SEN_FUNNEL issue in MATLAB (§5 / TASK D) —
> MATLAB runs on Ubuntu.** The PX4-SITL tasks (§2–§4) run **alongside**. Both implementations share
> the *same* SEN_FUNNEL demand-starvation bug, so converging it in MATLAB (fast, deterministic, no
> SITL flakiness) and porting the validated mechanism to PX4 is the efficient path. Start with §5.

---

## 0. Sync first

```bash
cd ~/Soft-Precise-Landing
git pull origin main          # export PATH=$PATH:"/c/Program Files/GitHub CLI" only if gh missing
```

This brings the MATLAB CBF/SEN port (`MATLAB/Common/cbf2_filter.m`, `MATLAB/Multi_init_cond/
visualControl_IBVS_adaptive.m`) and this handoff. The MATLAB changes are reference only — they do
not affect PX4 SITL.

---

## 1. Context — what the MATLAB session established (all validated in MATLAB sim)

The PX4 `cbf2` visibility CBF + SEN_FUNNEL were ported into the MATLAB Phase-1 controller. Four
findings came out of it that matter for PX4:

1. **Fix B thrust convention — PX4 is already correct.** Using `th_safe` directly for `R_d` only
   works if the thrust divides by the **measured/actual** tilt. PX4 does this (`B_T = m·(I_a[2]+g)/
   cos(euler)cos(euler)`, `controller.py:1284`, using the measured `euler`). A MATLAB port that
   divided by the *commanded* tilt caused a touchdown limit cycle — PX4 never had that bug. No action.

2. **`N_z` κ_z runaway — PX4 is already correct.** PX4 runs `N_z=0.02` (not 0.05) "κ_z adaptation too
   fast on noisy flow" (`CONTROLLER_PARITY.md:61`). MATLAB had 0.05 and hit the documented §6 κ_z
   runaway under noise; set to 0.02 there too. PX4 needs nothing. No action.

3. **LPF placement — PX4 should adopt the MATLAB result.** See TASK A.

4. **SEN_FUNNEL demand-starvation — shared bug, fix it here.** See TASK B.

---

## 2. TASK A — Merge the LPF-before-CBF change, get fresh results, compare (PRIMARY)

**Why.** In PX4, Fix B builds the desired attitude from the *unfiltered* `th_safe` (the CBF runs on
raw `I_a`, `controller.py:1106`; the `tau_ia` LPF is applied *after* at `:1146`). So PX4's attitude
**direction has no LPF** — exactly the state that gave MATLAB a hard, noisy touchdown. The MATLAB fix:
**move the LPF before the CBF** so the QP re-imposes the hard FoV bound on the *filtered* input —
noise cleaned, bound NOT smeared (filtering the CBF *output* would smear it). It composes with Fix B
and is roughly lag-neutral (it relocates the existing filter, doesn't add one). In MATLAB it recovered
the soft touchdown under noise while keeping the exact bound.

**Apply (env-gated, default-off — zero behavior change until `CBF_LPF_BEFORE=1`).** Two edits in
`PX4_Gazebo/src/controller.py`, inside `_attCtrl`:

*Edit A — right after the z-upright guard (`if I_a[2] >= 0: I_a[2] = -3.0`, ~line 1088):*
```python
        # Option 1: low-pass the DESIRED accel BEFORE the CBF, so the QP re-imposes
        # the hard FoV bound on the filtered input (clean attitude, bound NOT smeared).
        # Default off = current behavior (LPF after the CBF). Composes with Fix B.
        _lpf_before = os.environ.get("CBF_LPF_BEFORE", "0") == "1"
        if _lpf_before and len(self._I_a) > 0 and len(self._dt) > 0:
            _a = self._tau_ia / (self._tau_ia + self._dt[-1])
            I_a = _a * self._I_a[-1] + (1.0 - _a) * I_a
```

*Edit B — replace the `# ---- LPF (MATLAB tau_ia = 0.08 s) ----` block (~lines 1142-1147):*
```python
        # ---- LPF (MATLAB tau_ia = 0.08 s) ----
        if _lpf_before:
            self._I_a.append(I_a.copy())          # already filtered pre-CBF; store CBF-constrained result
        elif len(self._I_a) == 0:
            self._I_a.append(I_a.copy())
        else:
            alpha = self._tau_ia / (self._tau_ia + self._dt[-1])
            self._I_a.append(alpha * self._I_a[-1] + (1.0 - alpha) * I_a)
```

**A/B (fresh results):**
```bash
cd ~/Soft-Precise-Landing/PX4_Gazebo
                  bash scripts/run_aruco_landing.sh    # baseline: LPF after CBF
CBF_LPF_BEFORE=1  bash scripts/run_aruco_landing.sh    # Option 1: LPF before CBF
# or the A/B harness (n reps each):  CBF_LPF_BEFORE=1 bash scripts/run_cbf_ab.sh
```

**Compare both performances on (NOT landing xy alone):**
- attitude chatter / `w_u` saturation rate,
- the lateral fly-away,
- **visibility** via `python3 tools/analyze_cbf_visibility.py` — should be unchanged (bound stays exact),
- soft/precise touchdown (clean GT vz, pre-contact — not the EKF post-contact value).

Adopt as default (`CBF_LPF_BEFORE=1`) only if it wins or is neutral on chatter/fly-away with visibility held.

---

## 3. TASK B — Diagnose + fix the SEN_FUNNEL demand-starvation in PX4

This is the documented `FUNNEL_CBF_DESIGN.md §9` "needs tuning" item, and the MATLAB clamp audit
pinned the exact mechanism (it reproduced there on the hardest IC: 62% of steps saturated).

**Mechanism.** When the normalized position error exceeds the funnel `p_s`, `S_s` clips at ±0.95, the
back-mapped PPC goes *gentle* at the bound, and `G_s⁻¹ ∝ p_s` collapses as the funnel contracts →
**lateral demand starves** → the drone can't close the offset → fly-away. The integral `izeta_s` winds
to its clamp alongside.

**Check it's happening in SITL.** Log/inspect over a fly-away run:
- `S_s` residency (`s_e_n / p_s`) pinned at ±(1−`S_MARGIN`),
- `izeta_s` pinned at ±`_izeta_clamp` (5.0).

**Where it lives in `controller.py`:**
- envelope `p_s`/`dp_s`: `_updatePerfFunc` ~551-555,
- `S_s → ζ_s → G_s → izeta_s → ζ̇_sd → V_ds_d`: `_updateImgFeatureParam` ~572-603; soft clip at the
  `S_MARGIN` line ~581,
- params: `PLASMC_PS0_{X,Y}` (1.2), `PLASMC_PSINF_{X,Y}` (0.35), `PLASMC_XIS_{X,Y}` (γ_s, 0.5);
  integral clamp `_izeta_clamp=5.0`,
- **reference fix already in the file:** the `h_e` funnel's Singhal hard outlier-containment
  (~686-700) — mirror it for `s_e_n`.

**Levers, ranked (from the clamp audit):**
1. **Funnel sizing (proximate):** widen `PLASMC_PS0` so a large initial error fits inside the funnel;
   and/or slow `PLASMC_XIS` (γ_s) so it doesn't contract past the error.
2. **§9 hard outlier-containment (structural):** on `|s_e_n/p_s| ≥ 1`, force `s_e_n` back onto the
   bound (mirror `controller.py:~686-700`) instead of the soft `S_MARGIN` clip.
3. **`PLASMC_KP/KI/KD` retune:** helps demand magnitude + the `izeta_s` windup (→ `KI`/clamp), but
   **can't overcome a saturated funnel alone** (`G_s⁻¹→0`).

A/B each candidate with `PLASMC_SEN_FUNNEL=1`, `n≥5` reps (single seeds are high-variance).

---

## 4. TASK C — Compare both performances (the deliverable)

Tabulate, across `n≥5` reps each:

| Config | soft+precise | lateral fly-away | visibility (hitilt_vis) | attitude chatter |
|---|---|---|---|---|
| baseline (LPF after, current SEN_FUNNEL) | | | | |
| + LPF before CBF | | | | |
| + SEN_FUNNEL fix | | | | |
| + both | | | | |

Pick the config that maximizes soft+precise without regressing visibility. The CBF only owns
visibility — judge it on that, never on landing xy.

---

## 5. TASK D — Resolve IC5 in MATLAB (MATLAB IS available on Ubuntu)

The deferred MATLAB failure (IC5 `[2,2,-3]` noisy fly-away) is the SAME SEN_FUNNEL demand-starvation
as TASK B. MATLAB runs on Ubuntu, so resolve it there directly — it's a faster, deterministic test
bed than SITL (fixed dt, repeatable, no flakiness), so it's a good place to converge the SEN_FUNNEL
mechanism before/with the PX4 SITL validation.

- Tune the SEN_FUNNEL in `MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m`: the block
  (`S_s/zeta_s/G_s/izeta_s/dzeta_sd/V_ds_d`) + params `K_ctrl.gamma_s, p_s_0, p_s_inf, izeta_s_max`,
  using the §3 lever ranking (funnel sizing → §9 hard outlier-containment → K_rp/ri/rd).
- Run + verify in MATLAB on Ubuntu:
  ```bash
  matlab -batch "global IC_OVERRIDE NOISE_OVERRIDE; IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=0; \
    cd('MATLAB/Multi_init_cond'); visualControl_IBVS_adaptive"     # IC5 noiseless first
  # then NOISE_OVERRIDE=1 for noisy; re-sweep the canonical 5 ICs [0,0,-5;2,2,-5;2,-2,-5;2,2,-7;2,2,-3].
  ```
  For a multi-IC/noisy sweep, drive the script from a MATLAB *function* wrapper (function scope
  survives the script's top-of-file `clear`; set IC/NOISE globals before each call, read the result
  vars or `temp1.mat` after). Judge IC5 by whether it lands + holds visibility (no fov_fail).
- Because the mechanism is shared, keep MATLAB and PX4 in sync: whichever side you converge first,
  port the validated SEN_FUNNEL *mechanism* to the other. Keep the intentional divergences (corner-
  based CBF in MATLAB vs centroid in PX4; see CONTROLLER_PARITY.md) — only mirror the SEN_FUNNEL.

---

## 6. Reference

- Design: `PX4_Gazebo/docs/CBF_visibility.pdf`, `FUNNEL_CBF_DESIGN.md` (§9 = SEN_FUNNEL), `CONTROLLER_PARITY.md`.
- Pure CBF: `PX4_Gazebo/src/cbf_visibility.py`; offline validator `tools/validate_cbf.py`.
- Visibility metric: `tools/analyze_cbf_visibility.py`. A/B harness: `scripts/run_cbf_ab.sh`.
- MATLAB counterpart (reference): `MATLAB/Common/cbf2_filter.m` (now corner-based — PX4 stays
  centroid-based: corners aren't reliably detected at altitude, hence the multi-marker board).
