#!/usr/bin/env python3
"""Analyze today's flight data to validate a_u_z clamp and s-coast-freeze fixes."""
import numpy as np
from pathlib import Path
import json

landing_dir = Path("L:/Claude/Soft Landing/Hardware/Test_Data/Landing/2026-08-03")
test_dirs = sorted([d for d in landing_dir.glob("Mon Aug  3*") if d.is_dir()])

print(f"Found {len(test_dirs)} test directories from today")
print("\n=== VALIDATION: a_u_z Descent Clamp ===")
print("Expected: a_u[2] (vertical acceleration) should be pinned at -0.2, -0.5, or -1.0 m/s²")

au_z_values = []
s_coast_status = []

for i, test_dir in enumerate(test_dirs, 1):
    control_file = test_dir / "Control_Data.npy"
    img_file = test_dir / "Img_Data.npy"
    
    if not control_file.exists():
        continue
    
    try:
        control_data = np.load(control_file, allow_pickle=True).item()
        a_u = control_data.get('a_u', np.array([]))
        
        if len(a_u) > 0 and a_u.ndim >= 2:
            a_u_z = a_u[:, 2] if a_u.shape[1] > 2 else []
            if len(a_u_z) > 0:
                min_z = np.min(a_u_z)
                max_z = np.max(a_u_z)
                au_z_values.append(min_z)
                
                # Check for clamp values
                unique_z = np.unique(np.round(a_u_z, 4))
                has_clamp = any(np.isclose(v, [-0.2, -0.5, -1.0]).any() for v in unique_z)
                
                print(f"Run {i:2d} ({test_dir.name}): min_z={min_z:.3f}, max_z={max_z:.3f}, "
                      f"clamp={'YES' if has_clamp else 'NO'}")
    except Exception as e:
        print(f"Run {i:2d}: Error reading {control_file.name}")

print(f"\n=== STATISTICS ===")
if au_z_values:
    print(f"Average min a_u_z: {np.mean(au_z_values):.3f} m/s²")
    print(f"Min of mins: {np.min(au_z_values):.3f} m/s²")
    print(f"Max of mins: {np.max(au_z_values):.3f} m/s²")

print("\n=== VALIDATION: s-coast-freeze Fix ===")
print("Expected: Lateral a_u_xy acceleration spikes should be reduced (2-6 m/s² vs 94-110 pre-fix)")

lateral_spikes = []
for i, test_dir in enumerate(test_dirs, 1):
    control_file = test_dir / "Control_Data.npy"
    if not control_file.exists():
        continue
    
    try:
        control_data = np.load(control_file, allow_pickle=True).item()
        a_u = control_data.get('a_u', np.array([]))
        
        if len(a_u) > 0 and a_u.ndim >= 2 and a_u.shape[1] > 1:
            a_u_xy = np.sqrt(a_u[:, 0]**2 + a_u[:, 1]**2)
            max_spike = np.max(a_u_xy)
            lateral_spikes.append(max_spike)
            
            if max_spike > 10:  # Flag high spikes
                marker = "⚠️  HIGH"
            else:
                marker = "✓"
            print(f"Run {i:2d}: max lateral a_u = {max_spike:.2f} m/s² {marker}")
    except Exception as e:
        pass

if lateral_spikes:
    print(f"\nAverage max lateral spike: {np.mean(lateral_spikes):.2f} m/s²")
    print(f"Min spike: {np.min(lateral_spikes):.2f} m/s²")
    print(f"Max spike: {np.max(lateral_spikes):.2f} m/s²")
    print(f"Runs with spike > 10 m/s²: {sum(1 for s in lateral_spikes if s > 10)}")

