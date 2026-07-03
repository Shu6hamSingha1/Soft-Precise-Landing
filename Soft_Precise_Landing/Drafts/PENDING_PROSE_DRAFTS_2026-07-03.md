# Pending manuscript prose drafts — 2026-07-03 (awaiting user approval)

Status: the MECHANICAL refresh is applied+committed (625fc84): Tables III/IV rows, abstract,
contribution bullet, §IV multi-init/speed/pixel numbers, comparison item 1, conclusion bullets.
The passages below have CHANGED CLAIMS (not just numbers) and were drafted for review per the
drafts-before-apply rule. Apply after approval; then re-render any affected caption.

Sources for every number: scripts/_current_numbers.json (2026-07-03 regen at final code),
fresh per-baseline failure signatures (all daggered cells = FoV breaks; no 40s timeouts),
fresh IC2/Case-3 internals stats (from Sinusoidal_multi_init.mat run 2).

Closed without action: no p_r_inf>=1 condition exists in the manuscript (Standing-Cond worry was
from the old control_formulation draft); gains table already at LOCKED.

---

## D1 — §II image orientation (eq `image orientation: equation` + sentence, ~line 188-192)
REPLACE the definition sentence + post-equation sentence with:

The \emph{image orientation} $\alpha$ is extracted from weighted second-order centered image
moments of the $N$ image feature points $\hat{\boldsymbol{r}}_i$,
[equation unchanged: alpha = 1/2 tan^{-1}(2 mu_11/(mu_20 - mu_02))]
with $\mu_{pq} = \sum_{i=1}^{N} w_i(\hat{x}_i - \hat{x}^w)^p(\hat{y}_i - \hat{y}^w)^q$, where
$\hat{x}^w,\hat{y}^w$ are the components of the weighted centroid and the weights $w_i>0$ are
distinct per feature point. Even-order moments are invariant under a half-turn rotation, so the
principal axis alone defines $\alpha$ only modulo $\pi$. The distinct weights displace the
weighted centroid from the unweighted one. The displacement is a first-order moment that rotates
one-to-one with the target. Selecting the axis end aligned with the displacement renders $\alpha$
measurable on the full turn $(-\pi,\pi]$.

PLUS one clause in §IV setup (~line 498): "...tracking $N=4$ feature points with orientation
weights $w=[4,3,2,1]$."

## D2 — §III desired image velocity (¶268 tail + eq `h_d final` + ¶272)
REPLACE "The desired image velocity combines the measured centroid rate..." through the
post-equation sentence with:

The desired image velocity combines the funnel-prescribed centroid rate with the rotation and
descent feedforward,
\begin{equation} \label{h_d final: equation}
    \boldsymbol{h}_\text{d} = \begin{bmatrix}\boldsymbol{\varphi}_{\text{max},xy}\odot\bar{\boldsymbol{S}}_r\odot\dot{\boldsymbol{p}}_r\\ 0\end{bmatrix} + \dot{\psi}_\text{b}\,\hat{\boldsymbol{e}}_3\times\boldsymbol{s} + h_\text{rd}\,\boldsymbol{s},
\end{equation}
where $\bar{\boldsymbol{S}}_r=\bar{\boldsymbol{r}}_\text{e}\oslash\boldsymbol{p}_r$ is the
funnel-normalized position error and $\odot$ the componentwise product. The first term is the
centroid rate at which the position error rides the contracting funnel at constant
$\bar{\boldsymbol{S}}_r$. On the lateral axes the flow error
$\boldsymbol{h}_\text{e}=\boldsymbol{h}-\boldsymbol{h}_\text{d}$ therefore measures the departure
of the centroid rate from the funnel-consistent rate, equal to the scaled barrier-coordinate rate
$\boldsymbol{\varphi}_{\text{max},xy}\odot\mathcal{G}_r^{-1}\dot{\boldsymbol{\zeta}}_r$ up to the
descent-coupling residual. The sliding surface of the next subsection thus pairs the position
barrier $\boldsymbol{\zeta}_r$ with its own rate. The prescribed rate stays bounded even when the
error approaches the funnel boundary, where $\bar{\boldsymbol{S}}_r$ saturates, so a boundary
excursion enlarges $\boldsymbol{h}_\text{e}$ and summons recovery authority. The descent component
measures the departure of the radial flow from $h_\text{rd}$.

## D3 — §III c-term sentence (¶283, the s̈-drop text)
REPLACE "In $\dot{h}_d$ only the rotation and descent feedforward ... rather than differentiated." with:

In $\dot{\boldsymbol{h}}_\text{d}$ every term of the desired velocity is retained. The
funnel-prescribed centroid rate is smooth, so differentiating it introduces no measured centroid
acceleration; the $1/z$-inflated $\ddot{\boldsymbol{s}}$ of a measured-rate reference never
arises, and the residual measurement content is absorbed in the bounded disturbance
$\boldsymbol{d}_h$.

## D4 — yaw proof wrap sentence (~line 474)
REPLACE "The raw image-feature angle is unwrapped to $(-\pi/2,\pi/2]$ ... half-turn symmetry." with:

The weighted-moment disambiguation of the image orientation renders the raw angle measurable on
the full turn, and $\alpha_\text{e}$ is wrapped to $(-\pi,\pi]$ via
$\alpha_\text{e} = \text{atan2}(\sin\alpha_\text{e}^\text{raw},\cos\alpha_\text{e}^\text{raw})$.

(Also makes the proof consistent with Theorem 5's $\alpha_\text{e}(0)\in(-\pi,\pi]$.)

## D5 — per-IC narrative (¶ after Table III, ~line 592)
New facts: IC1 now WORST (2.52 cm); IC2 2.13; IC3 1.81; IC4 1.62 + t_max 11.04 s; IC5 2.37 +
t_max 8.96 s. Full replacement paragraph:

The four displaced ICs stress the closed loop in distinct modes. IC$_2=[2,2,-5]$~m (diagonal
lateral offset) drives the roll and pitch commands hardest in the first $1$--$2$~s, where the
visibility CBF conditions the commanded tilt while $\boldsymbol{\kappa}(t)$ rises to absorb the
combined lateral chase, yet lands within $2.13$~cm. The nominal IC$_1$, despite its zero initial
lateral offset, produces the largest terminal error of the set ($2.52$~cm), confirming that the
terminal phase is dominated by disturbance variance rather than initial geometry.
IC$_3=[2,-2,-5]$~m, the sign-reversed counterpart of IC$_2$ across $y$, lands within $1.81$~cm
and confirms no hidden $y$-axis bias. The high-altitude IC$_4=[2,2,-7]$~m extends the descent the
most, to $t_\text{f}^{\max}=11.04$~s. The features start depth-scaled by $5/7$, so the visibility
constraint is slack at $t=0$ and the image-position error contracts toward its terminal bound
while the UAV still has altitude, holding the terminal error to $1.62$~cm. The low-altitude
IC$_5=[2,2,-3]$~m is the tightest visibility case and the fastest to land
($t_\text{f}^{\max}=8.96$~s). The pinhole projection magnifies the lateral offset by
$5/3\approx 1.67\times$, producing the worst-case pixel excursions of the $25$-run set, with the
visibility barrier most active in the first $1$--$2$~s and the FoV margin remaining strictly
positive throughout. Across the five ICs, landing time follows initial altitude while touchdown
error tracks terminal-phase disturbance variance more than initial geometry.

## D6 — comparison observations item 2 (~line 691)
Full replacement (all daggered cells are FoV breaks; Zhang lands Cases 1/2/5):

Each baseline fails on a \emph{structural} ground that within-framework retuning cannot close.
\cite{lin2022} saturates its single PPC barrier under mean wind and mean-target motion, forcing
image features outside the camera FoV within $3.7$--$9.0~\text{s}$ on every trajectory,
$1.1$--$2.6~\text{m}$ above the surface. \cite{zhang2026} reaches the target surface on Cases~1,
2, and~5 but lands neither precise nor soft, with terminal lateral errors of
$0.19$--$0.26~\text{m}$, $2$--$3\times$ the $0.08~\text{m}$ precise bound, and terminal speeds of
$0.81$--$0.90~\text{m/s}$, over $4\times$ the $0.20~\text{m/s}$ soft bound; on Cases~3 and~4 its
fixed-timescale descent leaves the lateral loop lagging until the features leave the FoV
$0.29$--$0.43~\text{m}$ above the surface. \cite{lin2023} carries no disturbance-rejection
integrator, so under wind it holds a steady lateral image offset that no within-framework gain
choice removes; the off-center target leaves the camera FoV $0.9$--$7.9~\text{s}$ after launch on
every trajectory, $1.6$--$4.9~\text{m}$ above the surface. \cite{cho2022} ties its lateral
velocity command and its adaptive altitude gate to the same centroid displacement, so the
throttled lateral chase lags the moving, wind-displaced target until the features leave the FoV
$5.4$--$6.4~\text{s}$ after launch, $1.4$--$2.1~\text{m}$ above the surface. The per-baseline
mechanism contrast against VDF-ASMC is summarized in Table~\ref{mechanism contrast: table}, with
the per-baseline failure analyses detailed in Appendix~\ref{baseline analyses appendix: section}.

## D7-D10 — appendix per-baseline paragraphs (§ Baseline analyses appendix)
Update to the same facts at appendix depth:
- Lin2022 (¶948): ranges -> breaks $3.74$--$8.96$~s, $1.06$--$2.58$~m (worst Case 4). The "peak
  lateral thrust T≈49 N on Case 2" micro-number is OLD-EPOCH — recompute from fresh .mat or drop.
- Zhang (¶951): "reaches surface on four moving cases 9.55–9.88 s" -> lands Cases 1/2/5 at
  7.38–7.60 s (xy 0.194–0.263, v 0.81–0.90); FoV break on Cases 3/4 at 0.29/0.43 m; "static breaks
  at launch (0.47 s)" claim is DEAD. Timescale-decoupling structural story survives.
- Lin2023 (¶954): "static descends softly, stalls z=0.22, xy=0.147 without touching down" is
  DEAD -> FoV break on ALL FIVE (0.89–7.94 s, 1.57–4.87 m up). "Extending to full 5-IC grid 0/25"
  is an OLD-EPOCH extension run — rerun (25 sims) or drop.
- Cho (¶957): "stalls 0.43–0.51 m, 40 s timeout on 4/5, Lissajous FoV at 20.66 s" is DEAD -> FoV
  break on ALL FIVE at 5.39–6.39 s, 1.39–2.12 m up. Gate-coupling mechanism prose retained as the
  cause of the lateral lag; the terminal symptom is now uniformly the visibility break.

## D11 — adaptive-gain figure paragraph (¶905)
New facts (fresh IC2/Case-3): kappa rises from kappa(0)=0.05 -> terminal kx 0.086 (peak 0.100),
ky 0.111 (peak 0.148), kz 0.106 — genuine online adaptation, replaces "stay near initial 0.125 /
0.25 unchanged". kappa_alpha decays 2.0 -> 5e-4 (no target heading on Case 3; leakage bleed).

## D12 — sliding figure paragraph (¶915)
REPLACE (old numbers are from a stale ~22 s epoch). New: reaching peaks |sx|=1.13, |sy|=1.53 at
the IC2 offset in the first instants; both inside E_xy=0.5 by t~1.7–1.9 s and remain (brief
noise excursions <=0.86); sz never leaves E_z=0.5 (peak 0.43 during the h_rd transient).
NOTE the old paragraph claims E_xy=1 — LOCKED is E=diag(0.5,0.5,0.5).

## D13 — thrust/accel figure paragraph (¶925) + ITS CAPTION (~921)
Thrust in [19.9, 20.9] N about mg=20.73 N. Lateral command peaks ~4.1 m/s^2 in the first 1–2 s
(after the one-step init transient), settles ~0.3 m/s^2. ⚠ The claim "stays strictly below the
cone-induced bound / saturation does not engage" is NOW FALSE: on the fresh run the command rides
the cone bound ~66% of the descent (min margin slightly negative, a filter-timing artifact).
Rewrite caption+paragraph as: the deliverable-tilt conditioning is ACTIVE — the command tracks the
cone-induced bound through the sustained chase, demonstrating the CBF shaping (Property 1 keeps
the realized tilt in the deliverable set).
