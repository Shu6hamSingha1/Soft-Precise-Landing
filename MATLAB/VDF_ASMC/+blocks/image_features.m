function [V_s, V_h, V_w, V_nP_i, cs] = image_features(C_nP, I_R_V, I_R_C, P, cs, C_ctr, w_body, valid)
%IMAGE_FEATURES  Scale-independent virtual-frame image parameters (tex Sec. II-B).
%   Given the measured corner pixels C_nP (2xN), de-rotates onto the virtual frame
%   V, forms the image Jacobian L_s, and recovers the image position s, orientation
%   alpha, and optic flow (h,w) via the pseudo-inverse  [h;w] = L_s^dagger * dP/dt
%   (tex eq. optic flow inverse). A causal Savitzky-Golay filter (window P.fw) cleans
%   the pixel-noise-amplified estimates. cs carries the per-step state (prev pixels,
%   raw-signal ring buffers, sample index).
%
%   Returns V_s = [s_x; s_y; 1; alpha], V_h (3x1 optic flow), V_w (3x1 angular).
%   Optional valid (1xN logical, run_simulation): points a real camera could not see (behind the image plane or outside the physical
%   frame) are NOT used. Flow rows need a point valid in BOTH the current and previous frame; alpha uses the currently valid points.
%   Line-sampled cross (cs.mk set, N>5): every sample is an identified point with full 2D image motion, so the flow solve is the
%   same 2-rows-per-point least squares as the 5-point cross, just over the visible samples; alpha is the circular mean described below.

    f = P.f;
    % The raw features (s_i, h_i, w_i) update ONLY on image-refresh steps and are HELD
    % between (the camera frame is decimated by ZOH). The SG buffer below then sees the
    % held staircase, NOT zeros on the 2/3 non-refresh steps — which would otherwise
    % ~3x under-report the optic flow and break the descent braking.
    refresh = (mod(cs.k-1, P.ZOH) == 0);
    if refresh
        % de-rotate camera rays onto the virtual (yaw-only) frame
        V_R_C  = I_R_V' * I_R_C;
        rays   = [C_nP; f*ones(1,size(C_nP,2))];
        vr     = V_R_C * rays;
        V_nP_i = f*vr(1:2,:)./vr(3,:);

        % image Jacobian L_s (point-feature interaction matrix), rows stacked per corner
        L_s = zeros(2*size(C_nP,2), 6);
        for j = 1:size(C_nP,2)
            x = V_nP_i(1,j); y = V_nP_i(2,j);
            L_s(2*j-1:2*j,:) = [f, 0, -x, -x*y/f, (f^2+x^2)/f, -y; ...
                                0, f, -y, -(f^2+y^2)/f,  x*y/f,  x];
        end
        N_   = size(C_nP,2);
        vmask = true(1, N_);
        if nargin >= 8 && ~isempty(valid), vmask = logical(valid(:))'; end
        lines = isfield(cs, 'mk') && ~isempty(cs.mk) && N_ > 5;
        if lines
            if nnz(vmask) >= 3
                V_s_i = image_feature(V_nP_i(:,vmask)/f, cs.mk.wq(vmask));   % alpha from the VISIBLE samples only
            else
                V_s_i = cs.V_s_i;  if cs.k == 1, V_s_i = [0;0;1;0]; end       % <3 visible samples: hold
            end
        else
            V_s_i = image_feature(V_nP_i/f);             % [centroid; 1; alpha]  (Common/)
        end
        if nargin >= 6 && ~isempty(C_ctr)            % optional marker CENTRE (arm intersection) replaces the point-mean centroid
            vrc = V_R_C*[C_ctr(:); f];
            V_s_i(1:2) = vrc(1:2)/vrc(3);            % = (f*vrc(1:2)/vrc(3))/f
        end
        if lines && any(vmask)
            % Line-sampled cross: alpha = radius-weighted circular mean of [atan2(sample - centre) - line direction] over the VISIBLE samples.
            % NOT the PX4 estimator (unweighted 2nd-moment axis over the detected pixels + stub-vs-arm disambiguation): that is fine while the
            % whole marker is in view but, once the frame clips a large marker, the window's own shape imposes the principal axis (measured
            % 0.215 rad rms error vs the true relative yaw below 1.5 m, and the wrong sign at times). This estimator is unbiased for any visible
            % subset (0.004-0.012 rad rms vs true yaw at every altitude), and gives the full +-pi range because each sample knows its line.
            dd_ = V_nP_i(:,vmask) - V_s_i(1:2)*f;
            aa_ = atan2(dd_(2,:), dd_(1,:)) - cs.mk.ang(vmask);  ww_ = sqrt(sum(dd_.^2,1));
            if sum(ww_) > 1e-9, V_s_i(4) = atan2(sum(ww_.*sin(aa_)), sum(ww_.*cos(aa_))); end
        end

        % flow-row selection: a point contributes only if valid now AND last frame
        vsel = vmask;
        if isfield(cs, 'pt_valid_prev'), vsel = vmask & cs.pt_valid_prev; end
        rs   = reshape([vsel; vsel], [], 1);
        few  = nnz(vsel) < 2 && isfield(cs, 'V_h_i') && cs.k > 1;   % <2 points -> 4 unknowns unobservable: hold previous solution

        % stacked pixel velocity dP/dt by finite difference over the image timestep
        V_2nP_i = reshape(V_nP_i, [], 1);
        if cs.k == 1
            dPdt = zeros(size(V_2nP_i));
        else
            dPdt = (V_2nP_i - cs.V_2nP_i_prev)/(P.dt*P.ZOH);
        end
        cs.V_2nP_i_prev = V_2nP_i;

        use_gyro = isfield(P, 'flow_reduced') && P.flow_reduced && nargin >= 7 && ~isempty(w_body) && isfield(cs, 'gy_R_V_prev');
        gv = 1; if isfield(P,'flow_gyro_variant'), gv = P.flow_gyro_variant; end
        if use_gyro && gv == 2
            % PX4-LITERAL variant (diagnostic): each frame leveled with its OWN attitude (dPdt above), A from the previous leveled points,
            % then b - A(:,4:5)*w_V(1:2) with w_V = prev-attitude V-frame rotation of the mean body rate (cross_marker_perception.py:2198).
            Pp = cs.V_nP_i_prev_mat; Lp = zeros(2*size(Pp,2),6);
            for j = 1:size(Pp,2)
                x = Pp(1,j); y = Pp(2,j);
                Lp(2*j-1:2*j,:) = [f, 0, -x, -x*y/f, (f^2+x^2)/f, -y; 0, f, -y, -(f^2+y^2)/f, x*y/f, x];
            end
            wV   = cs.gy_R_V_prev' * cs.gy_R_C_prev * (0.5*(w_body(:) + cs.gy_wb_prev(:)));
            sol4 = lsqminnorm(Lp(:,[1 2 3 6]), dPdt - Lp(:,[4 5])*wV(1:2));
            V_v_i = [sol4(1:3); wV(1:2); sol4(4)];
        elseif use_gyro
            % PX4 gyro de-rotation (cross_marker_perception.py:2192-2202): BOTH frames are leveled with the PREVIOUS attitude, so the
            % flow includes the camera's roll/pitch/yaw motion between frames; the gyro-known w_x,w_y (body rate averaged over the pair,
            % rotated into the previous V frame) are subtracted and [h_x h_y h_z w_z] solved by plain least squares. w_x,w_y are returned
            % as the gyro values (as in PX4).
            Rp   = cs.gy_V_R_C_prev;
            vrp  = Rp * rays;
            Pk   = f*vrp(1:2,:)./vrp(3,:);
            Pp   = cs.V_nP_i_prev_mat;
            Lp   = zeros(2*size(Pp,2), 6);
            for j = 1:size(Pp,2)
                x = Pp(1,j); y = Pp(2,j);
                Lp(2*j-1:2*j,:) = [f, 0, -x, -x*y/f, (f^2+x^2)/f, -y; 0, f, -y, -(f^2+y^2)/f, x*y/f, x];
            end
            b    = reshape(Pk - Pp, [], 1)/(P.dt*P.ZOH);
            wV   = cs.gy_R_V_prev' * cs.gy_R_C_prev * (0.5*(w_body(:) + cs.gy_wb_prev(:)));
            if few
                sol4 = [cs.V_h_i(1:3); cs.V_w_i(3)];
            else
                sol4 = lsqminnorm(Lp(rs,[1 2 3 6]), b(rs) + Lp(rs,[4 5])*wV(1:2));
            end
            if isfield(P, 'flow_omega_corr') && P.flow_omega_corr
                % h_meas = h - w_t x s (rigid flow referenced to the optical axis; the target rotating about its own origin, offset s
                % from the axis, adds a translation-like term). Recover the centre velocity: h = h_meas + w_t x s, w_t = w_z + camera yaw rate.
                w_t  = sol4(4) + wV(3);
                sc_  = V_s_i(1:2);
                sol4(1:2) = sol4(1:2) + w_t*[-sc_(2); sc_(1)];
            end
            V_v_i = [sol4(1:3); wV(1:2); sol4(4)];
        elseif isfield(P, 'flow_reduced') && P.flow_reduced
            % PX4 parity (cross_marker_perception._solve_jacobian): the full 6-unknown solve is geometrically
            % rank-deficient at a small point spread (h_x aliases w_y, h_y aliases w_x; measured corr -0.99), which the
            % truncated pinv only hides. Solve the REDUCED 4-unknown problem [h_x h_y h_z w_z] (columns 1,2,3,6) by plain
            % least squares, no truncation (= np.linalg.lstsq rcond=None -> lsqminnorm default tol). The leveled points
            % carry no camera roll/pitch and the target's roll/pitch rate is unknown, so w_x = w_y = 0 (not estimated).
            if few
                sol4 = [cs.V_h_i(1:3); cs.V_w_i(3)];
            else
                sol4 = lsqminnorm(L_s(rs,[1 2 3 6]), dPdt(rs));
            end
            V_v_i = [sol4(1:3); 0; 0; sol4(4)];
        else
            V_v_i = pinv(L_s, P.pinv_tol) * dPdt;        % [h; w] = L_s^dagger dP/dt
        end
        V_h_i = V_v_i(1:3);
        V_w_i = V_v_i(4:6);
        if cs.k == 1, V_dw_i = zeros(3,1); else, V_dw_i = (V_w_i - cs.V_w_i_prev)/(P.dt*P.ZOH); end
        cs.V_w_i_prev = V_w_i;
        if nargin >= 7 && ~isempty(w_body)
            cs.gy_R_V_prev = I_R_V; cs.gy_R_C_prev = I_R_C; cs.gy_V_R_C_prev = V_R_C;
            cs.gy_wb_prev = w_body(:); cs.V_nP_i_prev_mat = V_nP_i;
        end
        cs.pt_valid_prev = vmask;
        cs.V_s_i = V_s_i; cs.V_h_i = V_h_i; cs.V_w_i = V_w_i; cs.V_dw_i = V_dw_i; cs.V_nP_i = V_nP_i;
    else
        V_s_i = cs.V_s_i; V_h_i = cs.V_h_i; V_w_i = cs.V_w_i; V_dw_i = cs.V_dw_i; V_nP_i = cs.V_nP_i;
    end

    % causal Savitzky-Golay smoothing (mean for the warm-up window)
    cs.V_s_raw(:,cs.k)  = V_s_i;  cs.V_h_raw(:,cs.k) = V_h_i;
    cs.V_w_raw(:,cs.k)  = V_w_i;  cs.V_dw_raw(:,cs.k) = V_dw_i;
    W = P.fw;
    if cs.k < W
        V_s  = mean(cs.V_s_raw(:,1:cs.k), 2);
        V_h  = mean(cs.V_h_raw(:,1:cs.k), 2);
        V_w  = mean(cs.V_w_raw(:,1:cs.k), 2);
    else
        Vs = sgolayfilt(cs.V_s_raw(:,cs.k-W+1:cs.k),2,W,[],2); V_s = Vs(:,end);
        Vh = sgolayfilt(cs.V_h_raw(:,cs.k-W+1:cs.k),2,W,[],2); V_h = Vh(:,end);
        Vw = sgolayfilt(cs.V_w_raw(:,cs.k-W+1:cs.k),2,W,[],2); V_w = Vw(:,end);
    end
end
