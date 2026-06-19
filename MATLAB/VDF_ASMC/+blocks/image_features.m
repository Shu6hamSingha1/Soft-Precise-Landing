function [V_s, V_h, V_w, V_nP_i, cs] = image_features(C_nP, I_R_V, I_R_C, P, cs)
%IMAGE_FEATURES  Scale-independent virtual-frame image parameters (tex Sec. II-B).
%   Given the measured corner pixels C_nP (2xN), de-rotates onto the virtual frame
%   V, forms the image Jacobian L_s, and recovers the image position s, orientation
%   alpha, and optic flow (h,w) via the pseudo-inverse  [h;w] = L_s^dagger * dP/dt
%   (tex eq. optic flow inverse). A causal Savitzky-Golay filter (window P.fw) cleans
%   the pixel-noise-amplified estimates. cs carries the per-step state (prev pixels,
%   raw-signal ring buffers, sample index).
%
%   Returns V_s = [s_x; s_y; 1; alpha], V_h (3x1 optic flow), V_w (3x1 angular).

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
        V_s_i = image_feature(V_nP_i/f);             % [centroid; 1; alpha]  (Common/)

        % stacked pixel velocity dP/dt by finite difference over the image timestep
        V_2nP_i = reshape(V_nP_i, [], 1);
        if cs.k == 1
            dPdt = zeros(size(V_2nP_i));
        else
            dPdt = (V_2nP_i - cs.V_2nP_i_prev)/(P.dt*P.ZOH);
        end
        cs.V_2nP_i_prev = V_2nP_i;

        V_v_i = pinv(L_s, P.pinv_tol) * dPdt;        % [h; w] = L_s^dagger dP/dt
        V_h_i = V_v_i(1:3);
        V_w_i = V_v_i(4:6);
        if cs.k == 1, V_dw_i = zeros(3,1); else, V_dw_i = (V_w_i - cs.V_w_i_prev)/(P.dt*P.ZOH); end
        cs.V_w_i_prev = V_w_i;
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
