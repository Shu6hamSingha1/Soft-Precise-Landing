%--------------------------------------------------------------------------
%
% kappa_a_Solver: Defines the ordinary differential equations of
% the adaptive yaw gain kappa_a used in yaw sliding mode control
% kappa   - > Dependent Variable (adaptive gain)
% X       - > Independent Variable (sliding surface sigma_a)
% K       - > [n_a; p_a] adaptation law parameters
%
%--------------------------------------------------------------------------
function [dkappadt] = kappa_a_Solver(~, kappa, X, K)

% Constants
    n = K(1);
    p = K(2);

% Initial Condition
    sigma = X;

    % ODE
    dkappadt =  n * abs(sigma) - n * p * kappa;
end
