%% CB_ENGAGE_SWEEP  Find the tuning combo that ENGAGES funnel+kappa while HOLDING
%                   nominal precision/softness. (Stress-test the winner separately.)
%
% Scores several candidate configs on the nominal 15-cell set (Linear/Sinusoidal/
% Circular x IC1-5, realistic NOISE). Reports per-config: SP count, mean terminal
% xy & rel-vel (precision/softness), mean optic-flow engagement engH, mean position
% engagement engR, max kappa-rise dK. Lets us pick the combo on the eng-vs-precision
% frontier before escalating disturbance.
%
% Run:  cd MATLAB/Multi_init_cond; cb_engage_sweep
% Saves: ../Datasets/MultiInit/engage_sweep.mat
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;

% ----- candidate configs (all keep per-axis theta = current law) ---------------
C(1) = mkcfg('baked', struct());
% D: kappa lever ONLY (no funnel tightening) -- can N/kappa0 drive kappa alone?
C(2) = mkcfg('D_kappaONLY_k.05_N.10', struct( ...
        'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10])));
% A: midway between surviving C1 and cliff C2 (find the ceiling)
C(3) = mkcfg('A_Xi.5_phinf.5_Xir.2_prinf.85', struct( ...
        'Xi_h',diag([0.5,0.5,0.5]),'p_hinf',[0.5;0.5;0.85], ...
        'Xi_r',diag([0.2,0.2]),'p_rinf',[0.85;0.85], ...
        'kappa0',[0.05;0.05;0.10],'N',diag([0.06,0.06,0.06])));
% B: isolate p_hinf as the destabilizer -- C1 funnel rates but lower flow floor
C(4) = mkcfg('B_Xi.4_phinf.45_Xir.2_prinf.85', struct( ...
        'Xi_h',diag([0.4,0.4,0.4]),'p_hinf',[0.45;0.45;0.8], ...
        'Xi_r',diag([0.2,0.2]),'p_rinf',[0.85;0.85], ...
        'kappa0',[0.05;0.05;0.10],'N',diag([0.06,0.06,0.06])));

trajList = ["Linear","Sinusoidal","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
co = struct('NOISE',1,'GE',1,'delay',1);

R = struct('name',{},'sp',{},'n',{},'mxy',{},'mv',{},'mengH',{},'mengR',{},'maxdK',{},'cells',{});
for ci = 1:numel(C)
    sp=0; n=0; xy=[]; v=[]; eH=[]; eR=[]; dKall=[];
    fprintf('\n===== %s =====\n', C(ci).name);
    for t = 1:numel(trajList)
        for ic = 1:5
            r = onerun(C(ci).ov, p0, trajList(t), ic, co);
            d=r.data; idx=d.idx; P=d.P;
            engH = max(abs(d.V_h_e(:,1:idx))./max(d.p_h_log(:,1:idx),eps),[],2);
            engR = max(abs(d.s_e_log(:,1:idx)./P.phi_max(:))./max(d.p_r_log(:,1:idx),eps),[],2);
            dK   = max(d.kappa_log(:,1:idx),[],2)-P.kappa0(:);
            isSP = r.precise&&r.soft;
            sp=sp+isSP; n=n+1;
            xy(end+1)=r.final_xy; v(end+1)=r.final_rel_vel;            %#ok<AGROW>
            eH(end+1)=max(engH); eR(end+1)=max(engR); dKall(end+1)=max(dK); %#ok<AGROW>
            fprintf('  %-11s IC%d SP=%d xy=%.3f v=%.3f engH=%.2f engR=%.2f dK=%.3f\n', ...
                trajList(t),ic,isSP,r.final_xy,r.final_rel_vel,max(engH),max(engR),max(dK));
        end
    end
    R(ci)=struct('name',C(ci).name,'sp',sp,'n',n,'mxy',mean(xy),'mv',mean(v), ...
                 'mengH',mean(eH),'mengR',mean(eR),'maxdK',max(dKall), ...
                 'cells',struct('xy',xy,'v',v,'eH',eH,'eR',eR));
end

clear global VDF_OVERRIDE
fprintf('\n==================== SUMMARY (nominal 15-cell) ====================\n');
fprintf('%-42s  SP    mXY    mV   engH  engR  maxdK\n','config');
for ci=1:numel(R)
    fprintf('%-42s %2d/%-2d %.3f %.3f  %.2f  %.2f  %.3f\n', ...
        R(ci).name,R(ci).sp,R(ci).n,R(ci).mxy,R(ci).mv,R(ci).mengH,R(ci).mengR,R(ci).maxdK);
end
save(fullfile(fileparts(mfilename('fullpath')),'..','Datasets','MultiInit','engage_sweep.mat'),'R','C');
fprintf('\nSaved -> Datasets/MultiInit/engage_sweep.mat\n');

% ============================= local functions ===============================
function c = mkcfg(name, ov)
    ov.theta_per_axis = true;
    c = struct('name',name,'ov',ov);
end
function r = onerun(ov, p0, traj, ic, co)
    global VDF_OVERRIDE       %#ok<GVMIS>
    VDF_OVERRIDE = ov;
    x0 = [p0(ic,:)'; 1;0;0;0; zeros(3,1); zeros(3,1)];
    r  = run_simulation(x0, traj, [], 1.0, co, 1);
end
