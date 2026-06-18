---
name: Don't suggest the thrust+torque refactor
description: User has rejected the thrust+torque ROS 2 refactor multiple times; stop bringing it up
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---
**Rule:** Do not propose the PX4 thrust+torque ROS 2 refactor (the "manuscript-equivalent" architectural change that replaces MAVSDK body-rate with `VehicleThrustSetpoint`/`VehicleTorqueSetpoint` topics).

**Why:** Asked about it multiple times across 2026-05-20/21, deferred once explicitly ("hold off on the refactor") and then again ("Stop suggesting thrust-torque refactor"). Continuing to bring it up as the "right" structural fix is unhelpful — the user knows it exists and has weighed it against other priorities.

**How to apply:**
- When precision/stability tuning plateaus in the rate-mode SO(3) port, do NOT auto-recommend the refactor as the next step.
- Stay focused on what's tunable inside the current MAVSDK rate-mode architecture (gains, clamps, fallback behavior, image-processing parameters, filters, etc.).
- If genuinely no rate-mode path remains, state the plateau honestly and STOP — let the user direct the architectural conversation.
- The refactor is well-documented in the conversation log + commit history if the user ever wants to revisit it.

Plenty of useful work remains inside the rate-mode port: image-side improvements (LK window size, sensor cal), better marker-loss handling, alternate gain combinations, IC-set robustness, etc.
