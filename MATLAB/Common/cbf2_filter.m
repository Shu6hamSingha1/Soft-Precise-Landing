function [I_a, theta_cone, ok, th_safe, state] = ...
    cbf2_filter(I_a, R, R33, yaw_c, C_nP, f, phi_max, theta_cap, ...
                theta_cone, dt_last, refresh, w_rp, state)
% CBF2_FILTER  Target-visibility control barrier function (camera-plane theta-QP).
%
% MATLAB port of PX4_Gazebo/src/cbf_visibility.cbf2_filter (validated offline
% tools/validate_cbf.py 13/13 and in PX4 SITL on its own metric, target
% visibility). Design: PX4_Gazebo/docs/CBF_visibility.pdf.
%
% A camera-plane QP over the body tilt that keeps the marker on the sensor.
% Per cycle, from the measured centroid cr=(x,y) (tangent units C_nP/f), the
% current tilt, the yaw, and the desired lateral accel, project the desired
% image-axis tilt theta_d onto the FoV box by alternating projection, then
% apply a post-QP deliverability cap. Two-phase delta: Phase 1 (marker decodes)
% box half-width = phi_max only; Phase 2 (decode-fail, hysteresis 3, ramp 5
% frames) tightens it.
%
% Frame conventions (re-derived for MATLAB, confirmed via the L_w-fidelity
% check against this repo's forward projection, 5.6% vs identity's 216%):
%   * R = I_R_C (body/camera -> inertial DCM); camera = body, downward, NED.
%   * C_nP : 2xN measured corner pixels, ALREADY centre-relative (range +-res/2),
%     so the tangent feature is simply C_nP/f (no centre subtraction).
%   * phi_max = [res(1); res(2)]/(2f) in the SAME axis order as C_nP rows
%     (NB: this is NOT K.p_10, which is axis-swapped [res(2);res(1)]/2f).
%   * Rz(-yaw) maps inertial-horizontal -> image axes; theta is the LEAN vector
%     (a_xy/a_z), coupled to the feature through L_w*M (M = [0 1; -1 0]).
%
% INPUTS
%   I_a        3x1 desired inertial (NED) accel; z-upright guard (I_a(3)>=0 ->
%              -3) must already be applied by the caller. I_a(1:2) replaced.
%   R          3x3 I_R_C current body->inertial rotation.
%   R33        scalar R(3,3); arccos gives current tilt.
%   yaw_c      control yaw (rad).
%   C_nP       2xN measured corner pixels (centre-relative), or [] -> Phase 2.
%   f          focal length (px, scalar).
%   phi_max    2x1 FoV-edge tangent half-extent (= res/(2f), C_nP-axis order).
%   theta_cap  post-QP deliverable-tilt cap (rad).
%   theta_cone scalar tilt cone seed (used only by the Phase-2 fallback).
%   dt_last    effective image dt (s) for the drift/loom finite differences;
%              <=1e-6 disables them this cycle.
%   refresh    logical; true only on image-refresh steps (ZOH boundary). The
%              drift/loom previous-value trackers advance only on refresh so the
%              finite differences span the true image interval, not a ZOH-held
%              repeat.
%   w_rp       2x1 body roll/pitch rate [w_x; w_y] (rad/s) for the drift est.
%   state      struct persistent CBF state. Fields: delta_prev, ddelta_ref,
%              decode_fail_n, phase2_alpha, cr_prev, d, Lw2_prev. Pass the same
%              struct each cycle; it is returned updated.
%
% OUTPUTS
%   I_a        constrained command (I_a(1:2) replaced).
%   theta_cone commanded tilt magnitude (Phase 1) or tightened cone (Phase 2).
%   ok         true if the QP path ran (Phase 1, marker decoded); false on the
%              Phase-2 magnitude-clamp fallback.
%   th_safe    2x1 safe LEAN vector (image axes, theta_cap-clipped) on the
%              Phase-1 path, else []. The caller builds the desired attitude
%              directly: rd3 = [-Rz(yaw)*th_safe; 1] normalized.
%   state      updated persistent state.

% Tuned defaults (validated; the Python env knobs CBF_TAU / CBF_DMIN_EMA /
% CBF_PHASE2_HYSTERESIS / CBF_PHASE2_RAMP_FRAMES at their defaults).
tau         = 0.3;    % drift / loom look-ahead horizon (s)
ema         = 0.3;    % EMA factor on the drift estimate
fail_thresh = 3;      % consecutive decode-fails before Phase 2 activates
ramp_frames = 5;      % Phase-2 delta ramp length (frames)

a_z      = abs(I_a(3));
ok       = false;
th_safe  = [];
M90      = [0, 1; -1, 0];                 % omega = M90 * theta_lean

valid = ~isempty(C_nP) && all(isfinite(C_nP(:)));
if valid
    ct  = C_nP / f;                       % 2xN tangent (centre-relative already)
    cr2 = mean(ct, 2);                    % 2x1 board centroid (tangent)
    x2  = cr2(1); y2 = cr2(2);
    Lw2_base = [x2*y2, -(1 + x2*x2); 1 + y2*y2, -x2*y2];   % rotational interaction matrix (depth-free)
    delta2 = 0.5 * (max(ct, [], 2) - min(ct, [], 2));      % per-axis half-extent

    % --- loom rate (delta-dot), tracked for the Phase-2 tau*ddelta term ---
    if refresh
        if dt_last > 1e-6 && ~isempty(state.delta_prev)
            state.ddelta_ref = max((delta2 - state.delta_prev) / dt_last, 0.0);
        else
            state.ddelta_ref = zeros(2,1);
        end
        state.delta_prev = delta2;
    end

    % --- two-phase delta: Phase 1 = central marker decoded (here) ---
    % delta_eff = 0: centroid-only barrier; the marker is deliberately allowed
    % to grow and overflow as the UAV closes in. m2 = phi_max only.
    state.decode_fail_n = 0;
    state.phase2_alpha  = 0.0;
    m2  = max(phi_max, 1e-3);

    % --- exogenous (translation/target) drift, d = cr_dot_obs - L_w*omega_cur ---
    % Uses the RAW L_w (omega_rp is the rotation-axis rate, no M applied here).
    if refresh && dt_last > 1e-6 && ~isempty(state.cr_prev)
        d_raw   = (cr2 - state.cr_prev) / dt_last - Lw2_base * w_rp;
        state.d = (1 - ema) * state.d + ema * d_raw;
    end
    dft = tau * state.d;                  % held between refreshes (ZOH-aware)
    if refresh
        state.cr_prev = cr2;
    end
    state.Lw2_prev = Lw2_base;            % stash RAW L_w for Phase-2 headroom

    cz = cos(yaw_c); sz = sin(yaw_c);
    Rzm = [cz, sz; -sz, cz];              % Rz(-yaw): inertial -> image
    th_curr = Rzm * (-R(1:2,3) / max(abs(R33), 1e-3));   % current image-axis tilt
    th      = Rzm * (I_a(1:2) / max(a_z, 1e-6));         % theta_d = Rz(-yaw)*(a_xy/a_z)

    % lean-vector -> rotation-axis correction (L_w*M); validated 5.6% vs identity
    % 216%. CORNER-BASED CBF (MATLAB): with exact analytical corners we constrain
    % EVERY corner point to stay inside the FoV box, not just the centroid. PX4
    % guards the board centroid because individual corners are not reliably detected
    % at altitude (multi-marker board); MATLAB's corners are exact, so guarding them
    % directly makes the CBF guarantee match the strict per-corner fov_fail check
    % (phi_max = res/2f). The two-phase delta is then unnecessary (the corners ARE
    % the marker extent), and the centroid is used only for the drift estimate.
    Ncp      = size(ct, 2);
    Lw_c     = zeros(2, 2, Ncp);
    anchor_c = zeros(2, Ncp);
    for i = 1:Ncp
        xi = ct(1,i); yi = ct(2,i);
        Lwi = [xi*yi, -(1 + xi*xi); 1 + yi*yi, -xi*yi] * M90;   % per-corner coupling
        Lw_c(:,:,i)   = Lwi;
        anchor_c(:,i) = ct(:,i) - Lwi * th_curr + dft;          % f_i = cr_i + L_wi*(th-th_curr) + tau*d
    end
    for it = 1:10                         % alternating projection: keep EVERY corner in the box
        for i = 1:Ncp
            Lwi = Lw_c(:,:,i);
            ff  = anchor_c(:,i) + Lwi * th;
            for k = 1:2
                if ff(k) > m2(k)
                    r = Lwi(k,:)';
                    th = th - (ff(k) - m2(k)) / (r'*r + 1e-12) * r;
                    ff = anchor_c(:,i) + Lwi * th;
                elseif ff(k) < -m2(k)
                    r = Lwi(k,:)';
                    th = th - (ff(k) + m2(k)) / (r'*r + 1e-12) * r;
                    ff = anchor_c(:,i) + Lwi * th;
                end
            end
        end
    end

    % post-QP deliverability cap (outside the projection so it never creates
    % box infeasibility)
    tn = norm(th);
    if tn > theta_cap
        th = th * (theta_cap / tn);
    end
    th_safe = th;                         % safe LEAN vector (image axes) for direct->rd3
    I_a(1:2) = a_z * ([cz, -sz; sz, cz] * th);   % a_xy* = a_z*Rz(yaw)*theta*
    theta_cone = norm(th);                % commanded tilt magnitude
    ok = true;
end

if ~ok
    % --- Phase 2: central marker overflowed / decode failed ---
    % Hysteresis-gate (require fail_thresh consecutive fails) to suppress
    % flicker near touchdown; ramp delta_eff 0 -> last visible 1/2 ptp over
    % ramp_frames frames.
    state.decode_fail_n = state.decode_fail_n + 1;
    if state.decode_fail_n >= fail_thresh
        state.phase2_alpha = min(state.phase2_alpha + 1.0 / ramp_frames, 1.0);
    end
    p2_alpha = state.phase2_alpha;
    if ~isempty(state.delta_prev) && ~isempty(state.Lw2_prev) && p2_alpha > 0
        delta_eff  = state.delta_prev * p2_alpha;
        ddelta_eff = state.ddelta_ref * p2_alpha;
        m2_p2 = max(phi_max - delta_eff - tau * ddelta_eff, 1e-3);
        % per-axis headroom -> conservative theta tightening for the clamp
        cr_ref  = state.cr_prev;
        if isempty(cr_ref); cr_ref = zeros(2,1); end
        dft_ref = tau * state.d;
        eff_margin = max(m2_p2 - abs(cr_ref + dft_ref), 0.0);
        row_norms  = sqrt(sum(state.Lw2_prev.^2, 2));
        theta_tight = min(eff_margin ./ (row_norms + 1e-9));
        theta_cone  = min(theta_cone, max(theta_tight, 0.0));
    end
    % magnitude-clamp fallback (direction preserved)
    a_xy_lim = a_z * tan(theta_cone);
    a_xy_n   = norm(I_a(1:2));
    if a_xy_n > a_xy_lim && a_xy_n > 1e-9
        I_a(1:2) = a_xy_lim * I_a(1:2) / a_xy_n;
    end
end
end
