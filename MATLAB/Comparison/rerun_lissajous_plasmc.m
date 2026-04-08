% Run PLASMC on Lissajous 5 times to characterize variance.
for k = 1:5
    fprintf('\n========== TRIAL %d ==========\n', k);
    run_comparison(1, "Lissajous");
end
