# ⚠️ FALSE SP — flagged precise+soft, but the GT is degenerate

This rep — `BootstrapFix_n21/rep6` = `Landing_Test/Wed Jun 10 01-22-38 2026` (trial **NC48d**, gamma_s=1.0 + KAPPA0_Z=1.0 bootstrap) — is logged
`SoftPrecise{ precise: True, soft: True, xy_err: 5.7e-21, rel_vel: 0.134 }` but it is **NOT a real soft-precise landing.**

Why it is false (GT-verified):
- The GT `UAV Pose` is **frozen at the IC pose** `(0.153, -0.046, 5.012 m)` for the entire flight (~1516 frames / ~25 s) — the drone "never moves" in the log.
- At **frame 1517** the pose **snaps to the origin** `(0, 0, ~0)` and stays there to the end.
- `xy_err = 5.7e-21` is just the lateral distance from that **origin-reset** frame to the target at `(0,0)` — a numerical-zero artifact, **not** a dead-centre landing.
- The drone **never descended** (min logged altitude = 5.012 m until the reset).

**Cause:** a GT logging / pose-subscriber glitch (frozen → zeroed pose), not controller behaviour. The raw glitch is preserved in `Ground_Truth.npy`; this rep is marked FALSE in the bundle `summary.tsv` (precise/soft flipped to 0) so aggregators don't count it.

Discovered 2026-06-10. It was the **only** sub-10 cm rep in all 101 Jun-9/10 recordings — i.e. there is **no genuine SP in the saved honest-cal (R3) data**.
