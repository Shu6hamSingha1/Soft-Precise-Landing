---
name: PX4/Gazebo Python work lives inside the git project, copied from ~/ws/
description: Don't edit ~/ws/scripts/soft_precise_landing/ in place; copy active files into the Soft-Precise-Landing git project and edit there
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---
The user's working PX4/Gazebo Python pipeline lives at `/home/shubham/ws/scripts/soft_precise_landing/`. **Do not edit those files directly.** Instead, **copy the usable files into the Soft-Precise-Landing git project** and edit there. The originals at `~/ws/` are kept as a reference / running baseline.

**Why:** The user wants the active aligned/edited version under git version control (this project is the git repo), while the original Python that's known to fly in Gazebo stays untouched for A/B comparison and fallback.

**How to apply:**
- When asked to modify the PX4/Gazebo Python, FIRST copy the relevant files into `/home/shubham/Soft-Precise-Landing/` (under an appropriately named subfolder), THEN edit the copies.
- "Usable files" = the active, non-versioned scripts: `landing_test.py`, `controller.py`, `flight_controller.py`, `gz_subscriber.py`, `img_data.py`, `numerical_methods.py`, and any calibration scripts the user calls out. Skip the `_v0..v9` history, the `Old scripts/` folder, and the broken-import vestigial files (`imgstreamer.py`, `ground_truth.py`, `adaptive_controller_with_IC.py`).
- Confirm the destination subfolder with the user if one isn't obvious (e.g., `Soft_Precise_Landing/` already at the repo root may be the intended target).
- After editing, the user runs the new copies from the project tree; if anything breaks they can fall back to `~/ws/scripts/soft_precise_landing/` originals.
