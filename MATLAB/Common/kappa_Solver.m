%--------------------------------------------------------------------------
%
% zetaSolver: Defines the ordinary differential equations of 
% the zeta used to define Performance Constraints 
% y     - > Dependent Variable
% X     - > Independent Variables
%    
%--------------------------------------------------------------------------
function [dkappadt] = kappa_Solver(~, kappa, X, K, G)

% Constants
    N = K(1:3,:);
    P = K(4:6,:);

% Initial Condition
    sigma = X(1:3);
    theta = X(4:6);          % per-axis row-norm theta_k (3x1); scalar-replicated reproduces old law

    % ODE: diag(theta)* (not theta.*) folds the gain into N FIRST -> off-path BIT-EXACT to
    % old Theta_norm*N*G*abs(sigma) (FP non-assoc); per-axis bound when theta is a 3-vec.
    dkappadt =  diag(theta) * N * G * abs(sigma) - N * P * kappa;
end