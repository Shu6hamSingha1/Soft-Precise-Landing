Constants;

% %% Loading the best parameters and tuning selective parameters
% load("bestParam.mat");

%% Changed the below parameters with random functions
% *************************************************************************
% Control Parameters for vertical, lateral and heading control
% *************************************************************************
% % Defining Performance Function for Visibility Constraints
% K.gamma_1 = 2 * rand(2, 1);
% K.p_10 = [res(2)/2/f; res(1)/2/f];
% K.p_1inf = (0.01 + 0.75*rand(2, 1));

% *************************************************************************
% Defining Performance Function for Visibility Constraints
K.gamma_1 = [1;1];
K.p_10 = [res(2)/2/f; res(1)/2/f];
K.p_1inf = [0.01;0.01];

% Defining Image Feature Control Paramters
K.zp = diag((1 + 10*rand(1, 2)) .^ randi([-4, 1], 1, 2));
K.zi = diag((1 + 10*rand(1, 2)) .^ randi([-4, 1], 1, 2));
K.zd = diag((1 + 10*rand(1, 2)) .^ randi([-4, 1], 1, 2));

% % Defining Performance Function for Outlier Elimination
% K.gamma_2 = 2 * rand(3, 1);
% K.p_20 = (10 + 10*rand(3, 1)) .^ randi([1, 2], 3, 1);
% K.p_2inf = (0.1 + 10*rand(3, 1));

% Defining Performance Function for Outlier Elimination
K.gamma_2 = [1.2;1.2;1.5];
K.p_20 = [50.0;50.0;10.0];
K.p_2inf = [1.0;1.0;0.5];

% Defining Optical Flow Control Paramters
K.Omega = diag((1 + 10*rand(3, 1)) .^ randi([-4, 1], 3, 1));
K.Gamma = diag((1 + 10*rand(3, 1)) .^ randi([-4, 1], 3, 1));
K.P = diag((1 + 10*rand(3, 1)) .^ randi([-4, 1], 3, 1));    % No effect without disturbance
K.N = diag((1 + 10*rand(3, 1)) .^ randi([-4, 1], 3, 1));   % Do not change it.
K.kappa_0 = (1 + 10*rand(3, 1)) .^ randi([-4, 1], 3, 1);
K.E = diag(rand(3, 1));

% Defining Attitude Control Paramters
K.ep = diag((1 + 10*rand(1, 3)) .^ randi([-4, 1], 1, 3));
K.ei = diag((1 + 10*rand(1, 3)) .^ randi([-4, 1], 1, 3));
K.ed = diag((1 + 10*rand(1, 3)) .^ randi([-4, 1], 1, 3));

% Defining Inner-Loop Control Paramters
K.wp = diag((1 + 10*rand(1, 3)) .^ randi([-4, 1], 1, 3));
K.wi = diag((1 + 10*rand(1, 3)) .^ randi([-4, 1], 1, 3));
K.wd = diag((1 + 10*rand(1, 3)) .^ randi([-4, 1], 1, 3));

K.ff = diag((1 + 10*rand(1, 3)) .^ randi([-4, 1], 1, 3));