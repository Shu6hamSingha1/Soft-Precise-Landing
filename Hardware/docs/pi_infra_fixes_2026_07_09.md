# Pi Infrastructure Fixes — 2026-07-09 session

Reconstructed from `~/.bash_history` on the Pi. Covers everything besides the
fps/governor work already documented in `img_fps_fixes.md`.

## 1. SSH access repair

Multiple rounds of `sudo nano /etc/ssh/sshd_config` + `systemctl restart ssh`,
eventually landing on:
- `mkdir -p ~/.ssh` + appended the Windows machine's public key to
  `~/.ssh/authorized_keys` (`chmod 600` on the file, `700` on the dir) —
  this is what keyless SSH (`ssh doctor@192.168.0.161`) now depends on.
- Found and edited `/etc/ssh/sshd_config.d/50-cloud-init.conf` — cloud-init's
  drop-in config was overriding the main `sshd_config`'s
  `PasswordAuthentication`/`PubkeyAuthentication` settings (classic
  Debian/Ubuntu cloud-image gotcha: `sshd_config.d/*.conf` loads AFTER and
  wins over the base file).
- `sudo passwd doctor` + `sudo faillock --user doctor --reset` — password
  reset and a fail-lockout cleared at some point in the same troubleshooting
  pass.
- `sudo apt install xauth -y` — needed for X11 forwarding
  (`ssh -X`/interactive `display_camera.py` GUI windows); explains the
  "Warning: No xauth data; using fake authentication data" noise on every
  SSH command in this repo's session (harmless, xauth just isn't fully
  configured for non-interactive/headless SSH, but is now installed).

**Relevant to future sessions:** if SSH auth breaks again, check
`/etc/ssh/sshd_config.d/50-cloud-init.conf` FIRST, not just the main
`sshd_config` — this bit yesterday.

## 2. Python venv (`~/denv`) rebuilt

```
rm -rf /home/doctor/denv && python3 -m venv --system-site-packages /home/doctor/denv
/home/doctor/denv/bin/pip install --upgrade pip
/home/doctor/denv/bin/pip install numpy scipy opencv-python mavsdk
```

Full rebuild with `--system-site-packages` (needed so the venv can see the
apt-installed `picamera2`/`libcamera` Python bindings, which aren't
pip-installable in the normal way). This is why `~/denv` is the Pi's real
runtime venv with `ahrs`/`mavsdk`/etc — see
[[project_pi_output_cal]] "GYRO COMPENSATION" note: use
`~/denv/bin/python3`, never bare `python3`, for anything needing these libs.

Also installed separately later:
```
python3 -m pip install qtm==2.1.1
```
(the Qualisys mocap client library — needed by `output_calibration.py` /
`mocaptools.py` for GT during calibration takes).

## 3. Camera fps investigation trail (context for `img_fps_fixes.md`)

The history shows the actual experimentation that led to the raw-Bayer
capture fix:
1. Baseline BGR888 preview at 960x720 and 640x480 — capped well under 30 fps.
2. Tried explicit `controls={"FrameRate": 60}` on a video configuration —
   didn't help (ISP processing was still the bottleneck, not a requested-rate
   cap).
3. Checked `vcgencmd measure_clock arm/isp/v3d` + `get_throttled` +
   `scaling_governor` while debugging — this is where the CPU governor
   `ondemand`->`performance` sysfs write happened (see `img_fps_fixes.md`
   for why it didn't persist and the `rc.local` fix applied 2026-07-10).
4. Finally tested `create_video_configuration(raw={"size": (640,480)})` +
   `capture_array("raw")` directly — confirmed 30+ fps achievable by
   skipping the ISP entirely. This synthetic test is what got ported into
   `imgstreamer.py` as the dual-stream raw-Bayer design.

## 4. Scripts actually run/exercised yesterday

`display_camera.py`, `capture_calib_images.py`, `basic_flight_test.py`,
`find_hover_throttle.py`, `output_calibration.py` — all invoked multiple
times, interleaved with `sudo shutdown -h now` (physical reboots between
test rounds, likely for hardware/battery swaps).
