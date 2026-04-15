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
s_alpha = 1/2*atan2(2*mu_11, mu_20 - mu_02);
if abs(s_alpha) < 1e-03
    s_alpha = 0;
end

l_w = [((mu_20 - mu_02)*(2*mu_12 + P_g(2)*mu_11 + P_g(1)*mu_02) ...
    - 2*mu_11*(mu_21 + P_g(1)*mu_11 - mu_03 - P_g(2)*mu_02))/((mu_20 ...
    - mu_02)^2 + 4*mu_11 + 1e-06); ( - (mu_20 - mu_02)*(2*mu_21 ...
    + P_g(1)*mu_11 + P_g(2)*mu_20) + 2*mu_11*(mu_30 + P_g(1)*mu_20 - mu_12 ...
    - P_g(2)*mu_11))/((mu_20 - mu_02)^2 + 4*mu_11 + 1e-06); -1];

q = [P_g; 1; s_alpha];
end