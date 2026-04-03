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
    Theta_norm = X(4);
    
    % ODE
    dkappadt =  Theta_norm * N * G * abs(sigma) - N * P * kappa;
end