% Rerun all 5 controllers on Circular trajectory after rescaling
% (r=5.0, wz=0.11) and zf=0.2 sync with Multi_init_cond.
fprintf('\n##### Circular (rescaled) #####\n');
run_comparison(1:5, "Circular");
