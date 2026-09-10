% Re-run the 5-controller comparison on the 5-pt cross marker, fixed seed=1
% (matches the Multi_init IC2 convention used for the manuscript table).
trajs = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
for k = 1:numel(trajs)
    fprintf('\n##### %s #####\n', trajs(k));
    run_comparison(1:5, trajs(k), 1);
end
