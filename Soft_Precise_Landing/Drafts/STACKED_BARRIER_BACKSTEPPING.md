# Combined Sliding Surface from ζ_r and ζ_h (2026-06-16)

Naming: `_r` = feature (normalized image-point error r̄_e) control, `_h` = optic-flow control.
ζ_r = barrier of r̄_e (position, rel-deg-2);  ζ_h = barrier of the flow error h_e (rel-deg-1).

## Motivation
The back-mapped PID/PPC on ζ_r (the outer feature loop that produced the desired feature rate)
CANNOT ensure convergence of the normalized position error s_e_n = r̄_e: its marginal restoring
gain −(rp/2)g'(S_r) peaks at |S_r|=0.648 and goes ANTI-RESTORING beyond it, so the inward demand
collapses exactly where the error is largest (see [[feedback_sen_authority_analysis]]). We
therefore move ζ_r OUT of the weak outer PID and INTO the sliding surface, alongside ζ_h, so the
robust ASMC (reaching Γ + adaptive κ) drives ζ_r → 0 with sliding-mode authority through the
honest centroid signal.

The position enters the SLIDING SURFACE directly (NOT through a desired-rate PPC in h_d, and NOT
as a separate stacked surface dimension). This is the 3-D COMBINE (well-posed), not the 5-D stack.

## Plant cascade (relative degree)
```
a_d → ḣ = −β ᴿa_d + c̃_h + d_h              (flow h: rel-deg 1; PLANT uses c̃_h, the tilde)
h   → ṡ_e = h − w×s − [(h − w×s)·ê₃] s      (position s / r̄_e: rel-deg 2; flow drives it)
        error-dynamics coupling: c_h ≜ c̃_h − ḣ_d  (enters ζ̇_h below)
```

## Desired flow (no PPC desired-rate term)
```
h_d = w×s + [h_rd − (w×s)·ê₃] s         (rotation + descent feedforward only)
```
so h_e = h − h_d is the velocity-like error (deviation from the passive reference).

## Funnels
```
feature:  S_r = r̄_e ⊘ p_r,  ζ_r = ln((1+S_r)/(1−S_r)),  G_r = diag((e^{ζ_r}+1)²/(2e^{ζ_r}p_r))
          ζ̇_r = G_r(r̄̇_e − S_r ṗ_r),   r̄̇_e = ṡ_{e,xy}/φ_max   (MEASURED; rel-deg-2 → no a_d)
flow:     S_h = h_e ⊘ p_h,  ζ_h = ln((1+S_h)/(1−S_h)),  G_h = diag((e^{ζ_h}+1)²/(2e^{ζ_h}p_h))
          ζ̇_h = G_h(β u_h + c_h + d_h − S_h ṗ_h)            (carries the input u_h)
```

## New combined sliding surface
```
σ = ζ_h + χ ζ_aug ,   χ = diag(χ_r, χ_z) ,   ζ_aug = [ ζ_{r,k} ; ∫ζ_{h,z} ]
  (χ_r ∈ ℝ²ˣ² = lateral gain block,  χ_z = descent gain scalar;  blkdiag, NOT diag(χ_r,χ_r,χ_z))
  lateral (k=x,y):  σ_{xy} = ζ_{h,xy} + χ_r ζ_{r}     (honest position barrier ζ_r = integral-action)
  descent (z):      σ_z   = ζ_{h,z} + χ_z ∫ζ_{h,z}    (no z position funnel → time-integral of flow)
```
ζ_aug stacks the lateral position barriers ζ_{r,k} with the descent flow-integral ∫ζ_{h,z}; on the
lateral axes the position barrier ζ_r provides the integral-action toward zero using the HONEST
centroid (not the time-integral of the flow barrier).

## Well-posedness (vs the rejected 5-D stack)
```
σ̇ = ζ̇_h + χ ζ̇_aug ,   ζ̇_aug = [ ζ̇_{r,k} ; ζ_{h,z} ]   (since d/dt ∫ζ_{h,z} = ζ_{h,z})
   = G_h(β u_h + c_h + d_h − S_h ṗ_h) + χ ζ̇_aug
∂σ̇/∂u_h = β G_h   (3×3, INVERTIBLE) → relative degree 1, u_eq & κ-law defined.
```
- Input enters ONLY through ζ̇_h; the surface is 3-D with a square invertible interaction matrix.
- ζ_r is rel-deg-2 (ζ̇_r has no u_h) but is MEASURED → χ_r ζ̇_r is a known drift folded into u_eq,
  exactly like the descent χ_z ζ_{h,z} term. Position authority flows through ζ_h.
- CONTRAST: the 5-D stack σ=[ζ_r;ζ_h]+Χ∫[ζ_r;ζ_h] has matrix [0_{2×3};βG_h] (rank-3-in-5-D,
  position block unactuated — 3 inputs can't enforce a 5-D surface). That is the invalid one.

## Equivalent control / regressor (same ASMC machinery)
The drift `Χζ_2` of the current law becomes `χ ζ̇_aug`; u_sw, u_eq, leakage κ-law, cone clamp,
SO(3) unchanged. With ν̇ ≜ χ ζ̇_aug = [χ_r ζ̇_{r,k} ; χ_z ζ_{h,z}]:
```
θ    = [ −c_h + S_h ṗ_h − G_h⁻¹ ν̇,  I ]
u_eq = G_h[ −c_h + S_h ṗ_h − G_h⁻¹ ν̇ ]
u_sw = −Γσ − ‖θ‖ sat(E⁻¹σ) G_h κ
```

## Convergence
On the lateral manifold σ_k=0: ζ_{h,k} = −χ_r ζ_{r,k}. Since h_d is the rotation/descent
feedforward, ζ_h is the velocity-like coordinate, so σ = ζ_h + χ_r ζ_r is a (velocity, position)
PD-type surface; the reduced dynamics drive ζ_r → 0 (hence r̄_e → 0) through the honest centroid.
σ UUB (Theorem-1 machinery) ⇒ ζ_r, ζ_h UUB.

## What does NOT work (rejected, for the record)
- 5-D PI surface σ = ζ + Χ∫ζ with ζ=[ζ_r;ζ_h] (underactuated; see above).
- Treating ζ_h as exactly ζ̇_r: the loom/cross term ∝ s_xy makes h_e ≈ ṡ_e only approximately;
  the mismatch is a bounded perturbation absorbed by d_h / the adaptive κ.
See [[feedback_sliding_surface_relative_degree]], [[project_stacked_barrier_backstepping]].

## Status
Formulation only. control_formulation.tex NOT yet edited (awaiting go-ahead). Code already runs a
cascade (feature funnel → h_d → flow ASMC); this combined-surface form is a redesign (drop the
PPC desired-rate; add χ_r ζ_r to σ) that would need gated code + IC validation.
