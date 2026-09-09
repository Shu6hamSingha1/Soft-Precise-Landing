function [I_a_cd_filt, th_safe, theta_cone, cbf_ok, R33, cs] = cbf_visibility(I_a_cd, I_R_C, yaw, C_nP, B_w_c, P, cs)
%CBF_VISIBILITY  Target-visibility conditioning (tex eq. `cbf qp` / `cbf gz`).
%   Two-tier port of PX4 src/visibility_projection.py (docs/CBF_visibility spec,
%   9-Sep-2026). Keeps the marker's MEASURED camera-plane centre inside the real
%   FoV with a buffer, by the smallest change to the outer loop's commanded
%   specific thrust I_a_cd.
%
%   Tier 1 -- one convex QP each cycle: minimal outward-only lean projection with
%     the deliverability ball ||y|| <= sqrt(A_cap^2/a_z^2 - 1) folded in and a
%     penalised per-axis slack (rho) for graceful degradation. I_a_cd(3) is a
%     fixed input; only I_a_cd(1:2) is conditioned.
%   Tier 2 -- descent governor: scales only the DOWNWARD part of I_a_cd(3) by
%     g_z in [g_min,1] on a measured time-to-edge; self-releasing, no invariance
%     claim. Never commands a climb.
%
%   P.cbf_two_tier = false restores the legacy joint-QP (cbf2_filter) exactly.
%
%   FRAME KNOBS (verify against MATLAB's I_R_C convention before trusting a gate):
%     P.cbf_mount_deg  -- camera-mount yaw offset in  P_map = Rz(mount)*Rz(-yaw).
%     P.cbf_Le_sign    -- sign of L_e = cbf_Le_sign*(L_omega*M). Spec: -1 (a
%                         world-fixed point's image moves opposite the rotation).
%   UNVALIDATED in MATLAB -- run the IC / 50-cell gate.

    % ---- upstream LPF on the pseudo-inverse-amplified command (unchanged) ------
    cs.I_a_cd_filt = P.alpha_ia*cs.I_a_cd_filt + (1-P.alpha_ia)*I_a_cd;
    R33 = max(min(I_R_C(3,3), 1), -1);                       % measured tilt cosine

    % ---- legacy joint-QP fallback -------------------------------------------------
    if isfield(P,'cbf_two_tier') && ~P.cbf_two_tier
        refresh = (mod(cs.k-1, P.ZOH) == 0);  dt_img = P.ZOH*P.dt;
        if isfield(P,'jqp_on') && P.jqp_on
            jqp = struct('A_cap', P.A_cap, 'k_az', P.k_az, 'g', norm(P.g));
        else, jqp = []; end
        [cs.I_a_cd_filt, theta_cone, cbf_ok, th_safe, cs.cbf_state] = cbf2_filter( ...
            cs.I_a_cd_filt, I_R_C, R33, yaw, C_nP, P.f, P.phi_max_cbf, ...
            P.theta_cap, P.theta_cap, dt_img, refresh, B_w_c(1:2), cs.cbf_state, jqp);
        cs.I_a_cd_filt(3) = max(cs.I_a_cd_filt(3), P.a_floor);
        I_a_cd_filt = cs.I_a_cd_filt;
        return
    end

    g   = norm(P.g);
    a_z = abs(cs.I_a_cd_filt(3));  a_z = max(a_z, 1e-3);     % fixed input, never optimised

    % ---- measured marker centre (cross intersection = geometric centroid) ------
    r_t   = mean(C_nP, 2) / P.f;                             % camera-tangent units (px/f)
    b     = getfielddef(P,'cbf_buffer_frac', 0.15);
    phi   = (P.res(:)/2/P.f) * (1 - b);                      % [phi_1; phi_2], fixed camera constant

    % ---- feature response to the lean (tex eq. `Lw`; spec eq. lin) ------------
    x = r_t(1);  y = r_t(2);
    L_omega = [ x*y, -(1+x^2) ;  1+y^2, -x*y ];
    M       = [ 0 1 ; -1 0 ];                                % lean -> rotation-axis
    Le_sign = getfielddef(P,'cbf_Le_sign', -1);
    L_e     = Le_sign * (L_omega * M);                       % d c_next / d y

    mnt  = deg2rad(getfielddef(P,'cbf_mount_deg', 90));
    Rz   = @(a) [cos(a) -sin(a); sin(a) cos(a)];
    P_map = Rz(mnt) * Rz(-yaw);                              % inertial x-y -> image axes
    y_d    = P_map * (cs.I_a_cd_filt(1:2) / a_z);            % commanded lean, image axes
    y_curr = P_map * (-I_R_C(1:2,3) / R33);                  % realized lean, image axes

    % ---- moving-target lead  tau*d  (default inert: tau=0) --------------------
    tau = getfielddef(P,'cbf_drift_tau', 0);
    d   = zeros(2,1);
    if tau > 0 && isfield(cs,'cbf_drift') && numel(cs.cbf_drift) == 2
        d = cs.cbf_drift(:);                                 % caller supplies conditioned h_xy
    end
    lead = tau * d;

    % ---- Tier 1 QP:  min 1/2||y-y_d||^2 + rho/2||s||^2 ------------------------
    %   s.t. |c_next(y)|_k <= phi_k + s_k,  s>=0,  ||y|| <= y_max
    %   c_next(y) = r_t + L_e (y - y_curr) + lead.  Slack eliminated:
    %   s_k* = max(|c_next_k| - phi_k, 0)  ->  smooth strictly-convex penalty in y.
    rho    = getfielddef(P,'cbf_vis_rho', 2000);
    c0     = r_t - L_e*y_curr + lead;                        % c_next(y) = c0 + L_e*y
    if P.A_cap > a_z
        y_max = sqrt(P.A_cap^2/a_z^2 - 1);                   % deliverability ball
    else
        y_max = tan(P.theta_cap);                            % degenerate A_cap<=g: lean cap governs
    end
    y_s = y_d;                                               % warm start
    for it = 1:20                                            % projected Newton, constant Hessian/active-set
        c  = c0 + L_e*y_s;
        Hn = eye(2);  gn = (y_s - y_d);
        for k = 1:2
            v = abs(c(k)) - phi(k);
            if v > 0
                gk = sign(c(k)) * L_e(k,:).';
                gn = gn + rho * v * gk;
                Hn = Hn + rho * (gk * gk.');
            end
        end
        step = -(Hn \ gn);
        if norm(step) < 1e-10, break, end
        y_s = y_s + step;
        if norm(y_s) > y_max                                 % ball binds -> 1-D search on the circle
            th0 = atan2(y_s(2), y_s(1));  best = th0;  fb = inf;
            for th = th0 + linspace(-pi, pi, 121)
                yc = y_max*[cos(th); sin(th)];
                cc = c0 + L_e*yc;
                f  = 0.5*sum((yc - y_d).^2) + 0.5*rho*sum(max(abs(cc) - phi, 0).^2);
                if f < fb, fb = f; best = th; end
            end
            y_s = y_max*[cos(best); sin(best)];
            break
        end
    end
    y_star = y_s;
    s_star = max(abs(c0 + L_e*y_star) - phi, 0);
    cbf_ok = all(s_star <= 1e-6);

    % ---- back to inertial specific thrust; z passes through Tier 1 ------------
    I_a_xy = a_z * (P_map.' * y_star);
    cs.I_a_cd_filt(1:2) = I_a_xy;

    % ---- Tier 2: descent-rate governor (tex eq. `t edge` / `cbf gz`) ---------
    if ~isfield(cs,'cbf_r_prev') || isempty(cs.cbf_r_prev), cs.cbf_r_prev = r_t; end
    rdot = (r_t - cs.cbf_r_prev) / P.dt;  cs.cbf_r_prev = r_t;
    rem_k   = phi - abs(r_t);
    close_k = max(sign(r_t) .* rdot, 0);
    t_edge  = inf;
    for k = 1:2
        if close_k(k) > 1e-9, t_edge = min(t_edge, rem_k(k)/close_k(k)); end
    end
    T_react = getfielddef(P,'cbf_treact', 1.5);
    g_min   = getfielddef(P,'cbf_gmin', 0.2);
    g_z     = g_min + (1-g_min) * min(max(t_edge/T_react, 0), 1);
    if ~isfield(cs,'cbf_gz_prev') || isempty(cs.cbf_gz_prev), cs.cbf_gz_prev = 1; end
    a_gz    = getfielddef(P,'cbf_gz_lpf', 0.7);
    g_z     = a_gz*cs.cbf_gz_prev + (1-a_gz)*g_z;  cs.cbf_gz_prev = g_z;
    if cs.I_a_cd_filt(3) + g > 0                             % only while a descent is commanded
        cs.I_a_cd_filt(3) = (cs.I_a_cd_filt(3) + g)*g_z - g;
    end

    % ---- deliverability guards outside the QP (normally inert) ---------------
    cs.I_a_cd_filt(3) = max(cs.I_a_cd_filt(3), P.a_floor);
    az2 = abs(cs.I_a_cd_filt(3));
    if az2 > 1e-6                                            % repeat lean cap
        lean = norm(cs.I_a_cd_filt(1:2)) / az2;
        if lean > tan(P.theta_cap)
            cs.I_a_cd_filt(1:2) = cs.I_a_cd_filt(1:2) * (tan(P.theta_cap)/lean);
        end
    end
    na = norm(cs.I_a_cd_filt);                               % full-vector cap
    if na > P.A_cap, cs.I_a_cd_filt = cs.I_a_cd_filt * (P.A_cap/na); end

    % ---- outputs -----------------------------------------------------------------
    I_a_cd_filt = cs.I_a_cd_filt;
    th_safe    = M * y_star;                                 % body-axis lean for so3_tracker Fix-B
    theta_cone = P.theta_cap;                                % lean cap (logging)
end

function v = getfielddef(s, name, d)
    if isfield(s, name), v = s.(name); else, v = d; end
end
