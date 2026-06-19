addpath('../Common','../VDF_ASMC','../Multi_init_cond');
trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
ctrl_list = cell(1,25); kk=0;
for ti=1:5; for ii=1:5; kk=kk+1; ctrl_list{kk}={trajs{ti},ICs{ii}}; end; end
% run_simulation reference (clean function, unaffected by clearvars)
all_results = zeros(25,6);
for c=1:25
    o=run_simulation([ctrl_list{c}{2}(:);1;0;0;0;zeros(6,1)], ctrl_list{c}{1}, [],1.0,struct('NOISE',0),1000);
    all_results(c,4:6)=[o.final_xy,o.final_rel_vel,double(o.success&&o.precise&&o.soft)];
end
NOISE_OVERRIDE=0; SPEED_MULT=1.0; MC_SEED=1000; CTRL_SEL=1;
for c=1:25
    IC_OVERRIDE = ctrl_list{c}{2}(:);
    TRAJ_TYPE   = ctrl_list{c}{1};
    visualControl_comparison;
    all_results(c,1:3) = [norm(I_p_c(1:2)-x_t(1:2,idx)), norm(I_v_c-dx_t(1:3,idx)), double(landed)];
end
spc=0; spr=0; worst=0;
for c=1:25
    csp = all_results(c,3)&&all_results(c,1)<=0.08&&all_results(c,2)<=0.20;
    spc=spc+csp; spr=spr+all_results(c,6);
    dxy=abs(all_results(c,1)-all_results(c,4)); dvel=abs(all_results(c,2)-all_results(c,5));
    worst=max(worst,max(dxy,dvel));
    mm=''; if dxy>2e-3||dvel>2e-3, mm=' <-- MISMATCH'; end
    fprintf('CELL %2d | cmp %.3f/%.3f/%d | rsim %.3f/%.3f/%d | d %.4f %.4f%s\n', c, all_results(c,1),all_results(c,2),all_results(c,3), all_results(c,4),all_results(c,5),all_results(c,6), dxy,dvel,mm);
end
fprintf('>> cmp SP=%d/25  rsim SP=%d/25  worst|delta|=%.4f\n', spc, spr, worst);
