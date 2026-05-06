% One-off: Lissajous noiseless Run 5 with widened z-channel boundary layer
addpath(fullfile('..','Common'));

speed_mult = 1.0;
x0 = [2.0; -2.0; -5.0; 1; 0; 0; 0; zeros(6,1)];

cfg = struct('NOISE', 0, 'GE', 0, 'delay', 0);

Ez_trials = [0.5, 0.65, 0.75, 0.90, 1.0];
for Ez = Ez_trials
    K_ovr = struct('E', diag([1.0, 1.0, Ez]));
    fprintf('\n--- Ez=%.2f ---\n', Ez);
    run_simulation(x0, "Lissajous", K_ovr, speed_mult, cfg, 1005);
end
