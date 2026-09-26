# Flight test plan 2026-09-26 (fixed plan - no in-flight decisions)

Goal: confirm FIX-004..010 and FIX-015 (docs/FIX_LOG.md) with data. The pilot does not judge behaviour during flights; every command below is run in
order and all data is brought back. Each fix is isolated by an A/B arm; arm order is rotated across packs so battery state does not confound.

Every flight is labelled BEFORE it runs by the `fl` function (writes `Test_Data/Landing/plan_<DDMMYYYY>.tsv`: flight id, arm, pack, timestamp). Runs are matched
to FC logs by order; the .tsv resolves any aborted/failed runs.

| Arm | Env override | What it isolates |
|-----|--------------|------------------|
| A | none (all fixes on) | new behaviour (FIX-004,005,006,007,008,009,010,015) |
| B | `PLASMC_AU_FRAME=body` | FIX-004 OFF (legacy full-body-DCM a_u frame) |
| C | `PLASMC_THRUST_TILT_COMP=0` | FIX-005 OFF (legacy thrust law) |
| D | `PLASMC_AU_MAX_XY=0` | FIX-006 OFF (no lateral a_u cap) |
| E | `RATE_CORRECTION_WX=0.92 RATE_CORRECTION_WY=0.92` | FIX-017 test (roll/pitch rate correction 0.758/0.739 -> 0.92) |

Blocks (one fresh pack each, >= 22.5 V; A first and last in every block, middle arms rotated):
- Pack 1: A B C D E A   (F01-F06)
- Pack 2: A C D E B A   (F07-F12)
- Pack 3: A D E B C A   (F13-F18)
- Pack 4: A E B C D A   (F19-F24)
=> A x8, B/C/D/E x4 each = 24 flights.

## Commands (Pi) - every flight is ONE self-contained line (no shell function, no exports)
Setup, once per session (each is a separate command):
```bash
source ~/denv/bin/activate
cd ~/ws/scripts/precise_landing
md5sum controller.py flight_controller.py hardware_landing.py     # expect 99a48e58..., d7a03add..., d8301f1c...
rm -f Test_Data/.log_download_active
script -a Test_Data/Landing/console_$(date +%d%m%Y).txt            # transcript; type `exit` at the very end
```
Flights (paste one line at a time; each line first appends its label to `Test_Data/Landing/plan_<DDMMYYYY>.tsv`, then runs the flight):
```bash
# ===== PACK 1 (fresh pack >= 22.5 V) =====
echo -e "F01	A	pack1	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 python3 hardware_landing.py
echo -e "F02	B	pack1	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 PLASMC_AU_FRAME=body python3 hardware_landing.py
echo -e "F03	C	pack1	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 PLASMC_THRUST_TILT_COMP=0 python3 hardware_landing.py
echo -e "F04	D	pack1	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 PLASMC_AU_MAX_XY=0 python3 hardware_landing.py
echo -e "F05	E	pack1	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 RATE_CORRECTION_WX=0.92 RATE_CORRECTION_WY=0.92 python3 hardware_landing.py
echo -e "F06	A	pack1	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 python3 hardware_landing.py
# ===== PACK 2 (fresh pack >= 22.5 V) =====
echo -e "F07	A	pack2	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 python3 hardware_landing.py
echo -e "F08	C	pack2	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 PLASMC_THRUST_TILT_COMP=0 python3 hardware_landing.py
echo -e "F09	D	pack2	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 PLASMC_AU_MAX_XY=0 python3 hardware_landing.py
echo -e "F10	E	pack2	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 RATE_CORRECTION_WX=0.92 RATE_CORRECTION_WY=0.92 python3 hardware_landing.py
echo -e "F11	B	pack2	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 PLASMC_AU_FRAME=body python3 hardware_landing.py
echo -e "F12	A	pack2	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 python3 hardware_landing.py
# ===== PACK 3 (fresh pack >= 22.5 V) =====
echo -e "F13	A	pack3	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 python3 hardware_landing.py
echo -e "F14	D	pack3	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 PLASMC_AU_MAX_XY=0 python3 hardware_landing.py
echo -e "F15	E	pack3	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 RATE_CORRECTION_WX=0.92 RATE_CORRECTION_WY=0.92 python3 hardware_landing.py
echo -e "F16	B	pack3	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 PLASMC_AU_FRAME=body python3 hardware_landing.py
echo -e "F17	C	pack3	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 PLASMC_THRUST_TILT_COMP=0 python3 hardware_landing.py
echo -e "F18	A	pack3	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 python3 hardware_landing.py
# ===== PACK 4 (fresh pack >= 22.5 V) =====
echo -e "F19	A	pack4	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 python3 hardware_landing.py
echo -e "F20	E	pack4	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 RATE_CORRECTION_WX=0.92 RATE_CORRECTION_WY=0.92 python3 hardware_landing.py
echo -e "F21	B	pack4	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 PLASMC_AU_FRAME=body python3 hardware_landing.py
echo -e "F22	C	pack4	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 PLASMC_THRUST_TILT_COMP=0 python3 hardware_landing.py
echo -e "F23	D	pack4	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 PLASMC_AU_MAX_XY=0 python3 hardware_landing.py
echo -e "F24	A	pack4	$(date +%F_%T)" >> Test_Data/Landing/plan_$(date +%d%m%Y).tsv; PLASMC_HW_POS_FEEDBACK=1 IMG_RECORD=1 python3 hardware_landing.py
```
End of session:
```bash
exit
LOG_REMOTE_DATE_DIR=/fs/microsd/log/2026-09-26 /home/doctor/denv/bin/python3 -u download_flight_logs.py
```
Between flights (always, not conditional): disengage the RC kill switch before the next `fl` (a kill at the previous touchdown latches "Kill switch engaged");
swap to a fresh pack at each `# --- PACK n ---` marker; wait for "Cleanup complete" before typing the next line.

## Pull to Windows (Git Bash, repo root)
```bash
cd "/l/Claude/Soft Landing/Hardware/Test_Data"; mkdir -p Landing/2026-09-26 FlightLogs/2026-09-26
ssh doctor@192.168.1.110 "cd ~/ws/scripts/precise_landing/Test_Data/Landing && tar czf - 'Sat Sep 26'*/ Test_Videos/'Sat Sep 26'*.mp4 console_26092026.txt plan_26092026.tsv" | tar xzf - -C Landing/2026-09-26
ssh doctor@192.168.1.110 "cd ~/ws/scripts/precise_landing/Test_Data/FlightLogs && tar czf - 2026-09-26" | tar xzf - -C FlightLogs
```

## Analysis plan (per FIX_LOG entry; pass criteria live there)
- Arm A vs B (frame): vertical leak `(I_a_raw_z+9.81)-a_u_z`, flights with `I_a_raw_z > -5`, pilot takeover count, thrust in last 1 s vs hover/cos(tilt), touchdown speed.
- Arm A vs C (thrust): thrust vs hover/cos(tilt) while tilted, vertical speed gain in tilted phases.
- Arm A vs D (cap): terminal `a_u_xy`, terminal tilt, takeovers, lateral offset at handover.
- Arm A vs E (rate correction): achieved/intended rate gain (`rate_gain.py`), lateral tracking, oscillation.
- All arms: IMU touchdown (`[FC] IMU contact detected` line vs EKF height), arm-wait lines, failsafe messages, battery guard, transcript completeness.
- Scripts: `Hardware/Test_Data/analysis_2026-09-25/` (`common.py` loads any date); write the new results into docs/FIX_LOG.md `Result:` fields.
