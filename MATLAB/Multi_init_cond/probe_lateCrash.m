%% Deep probe: Lissajous R3 crash, funnel internals frame-by-frame
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));

p0_table = [0,0,-5; 2,2,-5; 2,-2,-5; 2,2,-7; 2,2,-3];
cfg = struct('NOISE',1,'GE',1,'delay',1);
k = 3;
x0 = [p0_table(k,:)'; 1;0;0;0; zeros(6,1)];

tmp = run_simulation(x0, "Lissajous", [], 1.0, cfg, 1000+k);
d   = tmp.data; idx_f = d.idx;

fprintf('\nCRASH at idx=%d t=%.3fs\n', idx_f, d.tRange(idx_f));
fprintf('wind=[%+.3f %+.3f]  dm=%+.3f  dJ=%+.3f  r_cog=[%+.4f %+.4f]\n', ...
    d.wind_mean(1), d.wind_mean(2), d.delta_m, d.delta_J, d.r_cog(1), d.r_cog(2));

% ======= Funnel p_1 vs V_s_e time history, last 1.5s =======
fprintf('\n--- p_1 vs V_s_e (position funnel) ---\n');
fprintf('  t      p_1x    p_1y    Vsex    Vsey   |S_1x|  |S_1y|   zeta_1\n');
win = max(1,idx_f-30):2:idx_f;
for ii = win
    fprintf('  %5.2f  %6.4f  %6.4f  %+6.4f %+6.4f  %6.3f  %6.3f   %6.3f %6.3f\n', ...
        d.tRange(ii), d.p_1(1,ii), d.p_1(2,ii), ...
        d.V_s_e(1,ii), d.V_s_e(2,ii), ...
        abs(d.V_s_e(1,ii)/d.p_1(1,ii)), abs(d.V_s_e(2,ii)/d.p_1(2,ii)), ...
        d.zeta_1(1,ii), d.zeta_1(2,ii));
end

% ======= Funnel p_2 vs V_h_e (velocity funnel) =======
fprintf('\n--- p_2 vs V_h_e (velocity funnel) ---\n');
fprintf('  t      p_2x    p_2y    p_2z    Vhex    Vhey    Vhez   |S_2x| |S_2y| |S_2z|\n');
for ii = win
    fprintf('  %5.2f  %6.3f  %6.3f  %6.3f  %+6.3f %+6.3f %+6.3f  %5.3f %5.3f %5.3f\n', ...
        d.tRange(ii), d.p_2(1,ii), d.p_2(2,ii), d.p_2(3,ii), ...
        d.V_h_e(1,ii), d.V_h_e(2,ii), d.V_h_e(3,ii), ...
        abs(d.V_h_e(1,ii)/d.p_2(1,ii)), ...
        abs(d.V_h_e(2,ii)/d.p_2(2,ii)), ...
        abs(d.V_h_e(3,ii)/d.p_2(3,ii)));
end

% ======= Commanded optic flow V_h_d =======
fprintf('\n--- V_h_d (commanded optic flow) & drone state ---\n');
fprintf('  t      Vhdx    Vhdy    Vhdz    altz    vx_b    vy_b    vz_b\n');
for ii = win
    fprintf('  %5.2f  %+6.3f %+6.3f %+6.3f  %6.3f  %+6.3f %+6.3f %+6.3f\n', ...
        d.tRange(ii), d.V_h_d(1,ii), d.V_h_d(2,ii), d.V_h_d(3,ii), ...
        abs(d.X_DS(3,ii) - d.x_t(3,ii)), ...
        d.X_DS(8,ii), d.X_DS(9,ii), d.X_DS(10,ii));
end

% ======= Target vs drone position, lateral error components =======
fprintf('\n--- Lateral position: drone vs target ---\n');
fprintf('  t     px      py      tx      ty      ex     ey\n');
for ii = win
    px = d.X_DS(1,ii); py = d.X_DS(2,ii);
    tx = d.x_t(1,ii); ty = d.x_t(2,ii);
    fprintf('  %5.2f  %+6.3f %+6.3f %+6.3f %+6.3f %+6.3f %+6.3f\n', ...
        d.tRange(ii), px, py, tx, ty, px-tx, py-ty);
end
