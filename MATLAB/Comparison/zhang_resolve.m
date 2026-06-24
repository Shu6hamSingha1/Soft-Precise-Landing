function zhang_resolve(seeds)
%ZHANG_RESOLVE  Baked-Zhang vs zSlow on the comparison harness, seed ensemble.
%   Resolves the zSlow/Lissajous call: runs Zhang (ctrl 3) on all 5 trajectories
%   across a noise-seed ensemble at the fixed comparison IC2, for two configs:
%     baked-old : Kc2_z=0.15, Kc3_z=0.5   (pre-zSlow)
%     zSlow     : Kc2_z=0.10, Kc3_z=0.35  (currently in InitGains)
%   Reports per-config land-rate + mean terminal xy/vrel over landed runs, and a
%   per-(traj) land tally, so the decision rests on the ensemble not seed 1002.
    if nargin<1||isempty(seeds), seeds=1002:1006; end
    addpath(fullfile(fileparts(mfilename('fullpath')),'..','Common'));
    global CMP_OVERRIDE SWEEP_STATE %#ok<GVMIS>
    cfgs = { 'baked-old', struct('Kc2',diag([2.0,2.0,0.15]),'Kc3',diag([2.5,2.5,0.5 ])); ...
             'zSlow',     struct('Kc2',diag([2.0,2.0,0.10]),'Kc3',diag([2.5,2.5,0.35])) };
    SWEEP_STATE = struct();
    SWEEP_STATE.cfgs  = cfgs;
    SWEEP_STATE.seeds = seeds;
    SWEEP_STATE.trajs = {"Static","Linear","Sinusoidal","Lissajous","Circular"};
    SWEEP_STATE.acc   = zeros(2,5);    % [land sp sumxy sumvrel total] per cfg
    SWEEP_STATE.land_by_traj = zeros(2,5);
    SWEEP_STATE.i = 0;
    SWEEP_STATE.njobs = 2*5*numel(seeds);
    fprintf('\n==== Zhang baked-old vs zSlow | seeds=[%s] @ IC2 ====\n', num2str(seeds));
    while SWEEP_STATE.i < SWEEP_STATE.njobs
        global CMP_OVERRIDE SWEEP_STATE %#ok<GVMIS,REDEFGI>
        SWEEP_STATE.i = SWEEP_STATE.i + 1;
        [ci,ti,sdi] = ind2sub([2,5,numel(SWEEP_STATE.seeds)], SWEEP_STATE.i);
        SWEEP_STATE.cur = [ci ti];
        CMP_OVERRIDE = SWEEP_STATE.cfgs{ci,2};
        CTRL_SEL = 3; TRAJ_TYPE = SWEEP_STATE.trajs{ti}; MC_SEED = SWEEP_STATE.seeds(sdi); %#ok<NASGU>
        visualControl_comparison;
        ci = SWEEP_STATE.cur(1); ti = SWEEP_STATE.cur(2);
        a = SWEEP_STATE.acc(ci,:); a(5)=a(5)+1;
        if exist('landed','var') && landed
            a(1)=a(1)+1; a(3)=a(3)+xy_err; a(4)=a(4)+rel_vel;
            SWEEP_STATE.land_by_traj(ci,ti) = SWEEP_STATE.land_by_traj(ci,ti)+1;
            if precise && soft, a(2)=a(2)+1; end
        end
        SWEEP_STATE.acc(ci,:) = a;
    end
    N = 5*numel(SWEEP_STATE.seeds);
    fprintf('\n%-10s LAND   SP   meanXY  meanVrel | per-traj land (Sta Lin Sin Lis Cir)\n','cfg');
    for ci=1:2
        a=SWEEP_STATE.acc(ci,:); lt=SWEEP_STATE.land_by_traj(ci,:);
        fprintf('%-10s %2d/%-2d  %2d/%-2d %5.3f   %5.3f   | %d %d %d %d %d\n', ...
            SWEEP_STATE.cfgs{ci,1}, a(1),N, a(2),N, a(3)/max(a(1),1), a(4)/max(a(1),1), lt(1),lt(2),lt(3),lt(4),lt(5));
    end
    CMP_OVERRIDE = [];
    fprintf('==== done ====\n');
end
