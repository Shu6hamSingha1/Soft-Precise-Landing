function [I_a, theta_cone, ok, th_safe, state] = ...
    cbf2_filter(I_a, R, R33, yaw_c, C_nP, f, phi_max, theta_cap, ...
                theta_cone, dt_last, refresh, w_rp, state, jqp)
% JQP (optional 14th arg) — PX4 PARITY PORT 2026-09-03. Struct with fields:
%   .A_cap  deliverable specific-thrust ceiling |I_a| <= A_cap  [m/s^2]
%   .k_az   descent-rate relief gain (PX4 CBF_AZ_COST_GAIN)     [m/s^2 per rad]
%   .g      gravity magnitude (hover is I_a(3) = -g)            [m/s^2]
% Omit it (13-arg call) or pass [] to keep the LEGACY theta-QP path, which is
% bit-identical to the pre-port behaviour. See the CBF_JOINT_QP block below.
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
% CBF_JOINT_QP gate (PX4 parity port 2026-09-03). Defined HERE, at function scope, so
% every branch (incl. the Phase-2 cone fallback) can test it; 13-arg callers and jqp=[]
% keep the legacy theta-QP path bit-identical.
jqp_on = (nargin >= 14) && ~isempty(jqp);

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
    % DRIFT_RECENTER (g, 1/s): seed the drift with the EXPECTED re-centering of
    % the centroid (heading from its offset cr2 toward image centre, -g*cr2), so
    % the CBF anticipates the -Y TRANSLATION its own lean produces and stops
    % stripping the re-centering lean. Addresses d=0-at-start myopia at the root.
    global DRIFT_RECENTER;
    d_eff = state.d;
    if ~isempty(DRIFT_RECENTER) && DRIFT_RECENTER > 0
        d_eff = state.d - DRIFT_RECENTER * cr2;
    end
    dft = tau * d_eff;                    % held between refreshes (ZOH-aware)
    if refresh
        state.cr_prev = cr2;
    end
    state.Lw2_prev = Lw2_base;            % stash RAW L_w for Phase-2 headroom

    cz = cos(yaw_c); sz = sin(yaw_c);
    Rzm = [cz, sz; -sz, cz];              % Rz(-yaw): inertial -> image
    th_curr = Rzm * (-R(1:2,3) / max(abs(R33), 1e-3));   % current image-axis tilt
    th      = Rzm * (I_a(1:2) / max(a_z, 1e-6));         % theta_d = Rz(-yaw)*(a_xy/a_z)
    th_d_dbg = th;                                        % (debug) desired lean before QP

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
    % ---- CBF_JOINT_QP (PX4 parity port 2026-09-03) --------------------------------
    % PX4 solves the FoV box directly for the FULL I_a (lateral AND vertical),
    % interleaved with the descent-rate relief and a projection onto the TRUE-thrust
    % deliverability sphere |I_a| <= A_cap. That removes the theta round-trip (compute
    % I_a -> normalise by a_z -> project -> un-normalise) and, crucially, makes a_z part
    % of the SAME iteration instead of a fixed input patched downstream. Mirrors
    % PX4_Gazebo/src/cbf_visibility.py (_joint_qp block), 6 outer x 5 inner passes.
    %
    % Frame note: PX4 folds the +90 deg into P = Rz_p90b*Rzm; MATLAB folds it into
    % Lw_c (Lwi = [..]*M90). Both give the same Lw*th product, and since M90 is a
    % rotation the relief weight ||th_desired - th_safe|| is invariant to which
    % convention is used -- so k_az transfers from PX4 unchanged.
    if jqp_on                       % gate defined at function scope, see top
        Ia_lat = I_a(1:2);          % start from the UNCONSTRAINED lateral command
        Ia_z   = I_a(3);            % thrust accel, hover = -g (same convention as PX4)
        Ia_z0  = I_a(3);            % unconditioned a_z, for the relief (no accumulation)
        % CONVERGENCE DIAGNOSTIC (added 2026-09-04, does not change any output). The fixed
        % 6-outer x 5-inner iteration budget was ported as-is from PX4 with NO residual check
        % -- an independent PX4 session the same day found this loop genuinely fails to
        % converge near touchdown (Memory/px4/project_joint_qp_nonconvergence_kappa_ratchet.md:
        % theta_cone chatters frame-to-frame, I_a swings, kappa ratchets, a_u detonates to
        % 1274 m/s^2). This tracks ||Ia_lat_new - Ia_lat_prev|| per outer iterate so a caller
        % CAN check state.jqp_converged after the fact; it does not itself change behavior --
        % the loop still runs the same fixed budget either way. See PX4_PARITY_PORT_SPEC.md.
        jqp_resid = zeros(1,6);
        for outer = 1:6
            Ia_lat_prev = Ia_lat;
            az_now = max(abs(Ia_z), 1e-6);
            M_c = zeros(2, 2, Ncp);
            for i = 1:Ncp
                M_c(:,:,i) = Lw_c(:,:,i) * Rzm / az_now;   % d(f_i)/d(I_a(1:2))
            end
            Ia_lat = project_box_Ia(Ia_lat, anchor_c, M_c, m2, Ncp);
            jqp_resid(outer) = norm(Ia_lat - Ia_lat_prev);

            % descent-rate relief: if the box suppressed lean, spend budget on TIME.
            % One-way by construction: max(.,-g) can bring a descent toward hover but
            % never past it into a climb; min(.) keeps lift already commanded.
            if jqp.k_az > 0 && Ia_z0 > -jqp.g
                th_safe_now = Rzm * (Ia_lat / az_now);
                lat_supp    = norm(th_d_dbg - th_safe_now);
                az_relieved = Ia_z0 - jqp.k_az * lat_supp;   % more negative = more lift
                Ia_z        = min(Ia_z, max(az_relieved, -jqp.g));
            end

            % TRUE-thrust deliverability sphere. I_a IS the thrust acceleration, so the
            % bound is |I_a| <= A_cap. (PX4's legacy |I_a + g*e3| bounded VEHICLE accel --
            % zero at hover, a ~72% over-permit -- fixed there 2026-09-03, commit 937db5a9.)
            T = norm([Ia_lat(:); Ia_z]);
            if T > jqp.A_cap && T > 1e-9
                sc     = jqp.A_cap / T;
                Ia_lat = Ia_lat * sc;
                Ia_z   = Ia_z   * sc;
            end
        end
        th = Rzm * (Ia_lat / max(abs(Ia_z), 1e-6));   % derived lean, for th_safe consumers
        % A converged solve makes the last outer iterate's residual small relative to the
        % first (the box/relief/sphere have stopped moving Ia_lat materially). Threshold is
        % a diagnostic choice, not load-bearing -- this NEVER feeds back into I_a/th/theta_cone.
        state.jqp_residual  = jqp_resid;
        state.jqp_converged = jqp_resid(end) < max(1e-3, 0.05 * jqp_resid(1));
    else
        th = project_box(th, anchor_c, Lw_c, m2, Ncp);   % LEGACY alternating projection
    end

    th_qp_dbg = th;                       % (debug) lean after QP projection
    % NOTE: the post-QP deliverable-tilt cap (theta_cap) has been MOVED OUT of the
    % CBF into the inner-loop SO(3) tracker (deliverable-tilt saturation, Property 1).
    % cbf2_filter now performs ONLY the visibility QP; theta_cap is retained in the
    % signature for call-site compatibility but is no longer applied here. The caller
    % clips th_safe to theta_cap before building R_d.
    th_safe = th;                         % safe LEAN vector (image axes) for direct->rd3
    % DIRECTIONAL INSET (DIR_INSET_RELAX px): restore the CBF's intended
    % directionality. If stripping the lean to the tight box REVERSED the
    % demanded inertial lateral sign, the constraint is killing a RE-CENTERING
    % demand (not a runaway) -> re-solve with the box widened by DIR_INSET_RELAX
    % toward the FoV edge, so the re-centering lean is delivered. Self-targeting:
    % only the reversal case (seed-6-like) triggers it; good seeds keep the inset.
    global DIR_INSET_RELAX;
    if ~isempty(DIR_INSET_RELAX) && DIR_INSET_RELAX > 0 && ~jqp_on   % legacy-path only
        Rzy_d   = [cz, -sz; sz, cz];                 % Rz(yaw): lean -> inertial
        axy_des = a_z * Rzy_d * th_d_dbg;
        axy_sf  = a_z * Rzy_d * th_safe;
        if any(axy_des .* axy_sf < 0)                % CBF reversed a lateral axis
            m2_relax = m2 + (DIR_INSET_RELAX / f);   % widen box toward FoV edge
            th_r = project_box(th_d_dbg, anchor_c, Lw_c, m2_relax, Ncp);
            th = th_r; th_safe = th_r;     % deliverable cap applied by caller (inner loop)
            global DIR_FIRES; if isempty(DIR_FIRES); DIR_FIRES = 0; end
            DIR_FIRES = DIR_FIRES + 1;                % (debug) count firings
        end
    end
    % STRIP-RATIO-GATED DRIFT (DRIFT_GATED g): self-targeting version of
    % DRIFT_RECENTER. Measure how much of the demanded lateral lean the tight
    % projection STRIPPED along the demand direction; apply the -g*cr2
    % re-centering anticipation SCALED by that strip fraction, then re-project.
    % Good seeds (-Y survives, strip~0) -> ~no relax; seed 6 (-Y stripped/
    % reversed, strip~1) -> full relax. Self-targets the seed-6 flip only.
    global DRIFT_GATED;
    if ~isempty(DRIFT_GATED) && DRIFT_GATED > 0 && ~jqp_on           % legacy-path only
        Rzg     = [cz, -sz; sz, cz];
        axy_des = a_z * Rzg * th_d_dbg;                  % desired inertial a_xy
        axy_tgt = a_z * Rzg * th_safe;                   % tight-projection result
        dmag    = norm(axy_des);
        if dmag > 1e-6
            delivered  = (axy_tgt' * axy_des) / dmag;    % component along demand
            strip_frac = max(0.0, 1.0 - delivered / dmag);  % 0 deliv, 1 stripped, >1 reversed
            if strip_frac > 0.05
                g_eff   = DRIFT_GATED * strip_frac;      % anticipation scaled by strip
                dft_g   = tau * (state.d - g_eff * cr2);
                anc_g   = zeros(2, Ncp);
                for ig = 1:Ncp
                    anc_g(:,ig) = ct(:,ig) - Lw_c(:,:,ig) * th_curr + dft_g;
                end
                th_g = project_box(th_d_dbg, anc_g, Lw_c, m2, Ncp);
                th = th_g; th_safe = th_g; % deliverable cap applied by caller (inner loop)
            end
        end
    end
    % SIGN-PRESERVING GUARD (CBF_NO_REVERSE): the CBF may shrink the lateral lean
    % toward 0 for visibility, but may NOT reverse its inertial sign. A reversal
    % means the CBF would command motion AWAY from the target (the seed-6 flip);
    % instead hold that axis at 0 (no inward push, but no runaway). Self-targeting
    % -- only fires when the projection actually reversed an axis.
    global CBF_NO_REVERSE;
    if ~isempty(CBF_NO_REVERSE) && CBF_NO_REVERSE && ~jqp_on         % legacy-path only
        Rzy_nr = [cz, -sz; sz, cz];                 % Rz(yaw): lean -> inertial
        axy_des = a_z * Rzy_nr * th_d_dbg;          % desired inertial a_xy
        axy_sf  = a_z * Rzy_nr * th_safe;           % CBF-safe inertial a_xy
        for jj = 1:2
            if axy_des(jj)*axy_sf(jj) < 0           % CBF reversed this axis
                axy_sf(jj) = 0.0;                   % hold, don't fly the wrong way
            end
        end
        th_safe = (Rzy_nr') * (axy_sf / a_z);       % map back to lean (Rz(-yaw))
        th      = th_safe;
    end
    % (debug, default-off) Does clipping -Y improve or hurt corner margin?
    % Identify the binding corner under the DESIRED lean th_d (most outside the
    % box), then test how a unit -Y inertial lean moves THAT corner on its
    % binding axis: same sign as the violation => -Y pushes it further out (clip
    % JUSTIFIED); opposite sign => -Y pulls it toward centre (clip HARMFUL).
    % Rows: [axy_d_y; axy_qp_y; marg_curr; marg_d; marg_qp; bind_val; bind_m2;
    %        negY_sens; justified(+1)/harmful(-1)]
    global CBF_DBG_ON CBF_DBG_LOG;
    if ~isempty(CBF_DBG_ON) && CBF_DBG_ON
        Rzy = [cz, -sz; sz, cz];
        axy_d_y  = a_z * (Rzy(2,:) * th_d_dbg);
        axy_qp_y = a_z * (Rzy(2,:) * th_qp_dbg);
        thY = Rzm * [0; -1];              % image-axis lean for a -Y inertial accel
        % worst-corner margins (m2 - |f_i|) over corners/axes
        mc = inf; md = inf; mq = inf; viol = -inf; bi = 1; bk = 1; bf = 0;
        for i = 1:Ncp
            fcur = anchor_c(:,i) + Lw_c(:,:,i)*th_curr;
            fd   = anchor_c(:,i) + Lw_c(:,:,i)*th_d_dbg;
            fq   = anchor_c(:,i) + Lw_c(:,:,i)*th_qp_dbg;
            for k = 1:2
                mc = min(mc, m2(k)-abs(fcur(k)));
                md = min(md, m2(k)-abs(fd(k)));
                mq = min(mq, m2(k)-abs(fq(k)));
                if abs(fd(k))/m2(k) > viol      % most-binding under desired lean
                    viol = abs(fd(k))/m2(k); bi = i; bk = k; bf = fd(k);
                end
            end
        end
        negY_sens = Lw_c(:,:,bi)*thY;  negY_sens = negY_sens(bk);
        justified = sign(negY_sens) * sign(bf);   % +1 -Y pushes binding OUT; -1 toward centre
        CBF_DBG_LOG(:, end+1) = [axy_d_y; axy_qp_y; mc; md; mq; bf; m2(bk); negY_sens; justified];
    end
    if jqp_on
        % Joint solve returns BOTH channels; a_z is an output here, not an input.
        I_a(1:2) = Ia_lat;
        I_a(3)   = Ia_z;
    else
        I_a(1:2) = a_z * ([cz, -sz; sz, cz] * th);   % a_xy* = a_z*Rz(yaw)*theta*
    end
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

function Ia = project_box_Ia(Ia, anchor_c, M_c, m2, Ncp)
% Alternating projection of the LATERAL COMMAND Ia onto the per-corner FoV box
% |f_i| <= m2, with f_i = anchor_i + M_i*Ia. Same algorithm as project_box but in
% I_a units (M_i = Lw_i*Rz(-yaw)/a_z), so a_z can vary between outer iterates.
% PX4 parity: cbf_visibility.py runs 5 inner passes per outer iterate.
for it = 1:5
    for i = 1:Ncp
        Mi = M_c(:,:,i);
        ff = anchor_c(:,i) + Mi * Ia;
        for k = 1:2
            if ff(k) > m2(k)
                r  = Mi(k,:)';
                Ia = Ia - (ff(k) - m2(k)) / (r'*r + 1e-12) * r;
                ff = anchor_c(:,i) + Mi * Ia;
            elseif ff(k) < -m2(k)
                r  = Mi(k,:)';
                Ia = Ia - (ff(k) + m2(k)) / (r'*r + 1e-12) * r;
                ff = anchor_c(:,i) + Mi * Ia;
            end
        end
    end
end
end

function th = project_box(th, anchor_c, Lw_c, m2, Ncp)
% Alternating projection of the lean th onto the per-corner FoV box |f_i|<=m2.
for it = 1:10
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
end
