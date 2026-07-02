% *************************************************************************
% Function to compute image features of a set of feature points nP
% *************************************************************************
function [q] = image_feature(nP)
%% Moments
% Zeroth Order
m_00 = moment(nP, 0, 0);

% First Order
m_10 = moment(nP, 1, 0);
m_01 = moment(nP, 0, 1);

% Second Order (For debugging)
m_20 = moment(nP, 2, 0);
m_02 = moment(nP, 0, 2);
m_11 = moment(nP, 1, 1);

% Centroid
P_g = [m_10; m_01]/m_00;

%% Centered Moments
% Zeroth Order (For debugging)
mu_00 = centered_moment(nP, P_g, 0, 0);

% Second Order
mu_20 = centered_moment(nP, P_g, 2, 0);
mu_02 = centered_moment(nP, P_g, 0, 2);
mu_11 = centered_moment(nP, P_g, 1, 1);

% Third Order
mu_30 = centered_moment(nP, P_g, 3, 0);
mu_21 = centered_moment(nP, P_g, 2, 1);
mu_12 = centered_moment(nP, P_g, 1, 2);
mu_03 = centered_moment(nP, P_g, 0, 3);

%% Computation of Image Features
% Orientation: weighted-corner 2pi-disambiguated principal angle (ported from
% the PX4 perception layer, _marker_principal_angle). The pi-period 2nd-moment
% axis 0.5*atan2(2 mu11, mu20-mu02) is invariant under 180deg; corner weights
% [4 3 2 1] shift the weighted centroid toward the high-weight corners, and that
% (weighted - geometric) centroid DISPLACEMENT is a 1st-moment vector that
% rotates 1:1 over a full turn. Flip the axis to the 180deg end aligned with it
% -> full +-180deg orientation (yaw observable past the old +-90deg fold).
N = size(nP, 2);
if N == 4, wq = [4, 3, 2, 1]; else, wq = ones(1, N); end
Wq  = sum(wq);
xcw = sum(wq.*nP(1,:))/Wq;   ycw = sum(wq.*nP(2,:))/Wq;     % weighted centroid
Xcw = nP(1,:) - xcw;         Ycw = nP(2,:) - ycw;
wm20 = sum(wq.*Xcw.*Xcw);  wm02 = sum(wq.*Ycw.*Ycw);  wm11 = sum(wq.*Xcw.*Ycw);
if abs(wm11) < 1e-9 && abs(wm20 - wm02) < 1e-9
    s_alpha = 0;
else
    s_alpha = 0.5*atan2(2*wm11, wm20 - wm02);               % pi-period axis
end
dxw = xcw - mean(nP(1,:));   dyw = ycw - mean(nP(2,:));     % weighted-centroid displacement
if dxw*dxw + dyw*dyw > 1e-18
    dd = s_alpha - atan2(dyw, dxw);
    if abs(atan2(sin(dd), cos(dd))) > pi/2, s_alpha = s_alpha + pi; end
end
s_alpha = atan2(sin(s_alpha), cos(s_alpha));                % wrap to (-pi, pi]

l_w = [((mu_20 - mu_02)*(2*mu_12 + P_g(2)*mu_11 + P_g(1)*mu_02) ...
    - 2*mu_11*(mu_21 + P_g(1)*mu_11 - mu_03 - P_g(2)*mu_02))/((mu_20 ...
    - mu_02)^2 + 4*mu_11 + 1e-06); ( - (mu_20 - mu_02)*(2*mu_21 ...
    + P_g(1)*mu_11 + P_g(2)*mu_20) + 2*mu_11*(mu_30 + P_g(1)*mu_20 - mu_12 ...
    - P_g(2)*mu_11))/((mu_20 - mu_02)^2 + 4*mu_11 + 1e-06); -1];

q = [P_g; 1; s_alpha];
end