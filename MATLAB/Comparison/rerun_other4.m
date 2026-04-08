% Rerun Static/Linear/Sinusoidal/Lissajous after zf=0.2 sync.
trajs = ["Static","Linear","Sinusoidal","Lissajous"];
for k = 1:numel(trajs)
    fprintf('\n##### %s #####\n', trajs(k));
    run_comparison(1:5, trajs(k));
end
