% Rerun all 5 controllers on all 5 trajectories with the S_2 guard fix.
trajs = ["Static","Linear","Sinusoidal","Circular","Lissajous"];
for k = 1:numel(trajs)
    fprintf('\n##### %s #####\n', trajs(k));
    run_comparison(1:5, trajs(k));
end
