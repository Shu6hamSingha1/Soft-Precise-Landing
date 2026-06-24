function sweep_baselines_deep(seeds)
%SWEEP_BASELINES_DEEP  Fresh best-effort deep sweep of the 4 baseline controllers.
%   sweep_baselines_deep(seeds) probes each baseline's binding gains around its
%   current InitGains value, over the 5 trajectories x a SEED ENSEMBLE at IC2,
%   via the CMP_OVERRIDE hook in visualControl_comparison. Baselines are
%   structurally 0-SP, so scoring is best-effort:
%     SP   : #soft-precise-landed / N
%     LAND : #reached z_gap<=0.2  / N
%     FoV  : #kept target in view / N
%     xy   : mean terminal lateral error over landed runs [m]
%     vrel : mean terminal rel. speed over landed runs [m/s]
%   [*] marks the current baked/kept InitGains value.
%
%   seeds : noise-seed ensemble at fixed IC2 (default [1 2 3]).

    if nargin < 1 || isempty(seeds), seeds = [1 2 3]; end
    addpath(fullfile(fileparts(mfilename('fullpath')),'..','Common'));

    % --- per-controller grids: {CTRL_SEL, label, override-struct or []} -----
    S = {};
    % Lin 2022 (ctrl 2): PPC PBVS -- track speed vs funnel
    S(end+1,:) = {2,'Lin2022 [*baked]', []};
    S(end+1,:) = {2,'Lin2022 k1=0.4',   struct('k1',0.4)};
    S(end+1,:) = {2,'Lin2022 k1=0.9',   struct('k1',0.9)};
    S(end+1,:) = {2,'Lin2022 k2=6',     struct('k2',6.0)};
    S(end+1,:) = {2,'Lin2022 rinfP_xy=0.5', struct('rho_inf_p',[0.5;0.5;0.15])};
    S(end+1,:) = {2,'Lin2022 lP_xy=0.02',   struct('l_p',[0.02;0.02;0.10])};
    % Zhang 2026 (ctrl 3): bandwidth vs descent timescale
    S(end+1,:) = {3,'Zhang [*baked]', []};
    S(end+1,:) = {3,'Zhang Kc2xy=1.5', struct('Kc2',diag([1.5,1.5,0.15]))};
    S(end+1,:) = {3,'Zhang Kc2xy=3.0', struct('Kc2',diag([3.0,3.0,0.15]))};
    S(end+1,:) = {3,'Zhang zSlow',     struct('Kc2',diag([2.0,2.0,0.10]),'Kc3',diag([2.5,2.5,0.35]))};
    S(end+1,:) = {3,'Zhang zFast',     struct('Kc2',diag([2.0,2.0,0.25]),'Kc3',diag([2.5,2.5,0.7]))};
    S(end+1,:) = {3,'Zhang Kc1xy=0.35',struct('Kc1',diag([0.35,0.35,0.03]))};
    % Lin 2023 (ctrl 4): IBVS feature-funnel -- centring vs descent
    S(end+1,:) = {4,'Lin2023 [*baked]', []};
    S(end+1,:) = {4,'Lin2023 k1xy=0.3', struct('k1',[0.3;0.3;0.40])};
    S(end+1,:) = {4,'Lin2023 k1xy=0.5', struct('k1',[0.5;0.5;0.40])};
    S(end+1,:) = {4,'Lin2023 k1z=0.3',  struct('k1',[0.4;0.4;0.30])};
    S(end+1,:) = {4,'Lin2023 k1z=0.55', struct('k1',[0.4;0.4;0.55])};
    S(end+1,:) = {4,'Lin2023 k2=2.5',   struct('k2',2.5)};
    % Cho 2022 (ctrl 5): current retuned -- lateral precision in FoV-safe band
    S(end+1,:) = {5,'Cho [*kept]', []};
    S(end+1,:) = {5,'Cho Kvxy=1.2', struct('Kv',diag([1.2,1.2,2.0]))};
    S(end+1,:) = {5,'Cho Kvxy=2.5', struct('Kv',diag([2.5,2.5,2.0]))};
    S(end+1,:) = {5,'Cho lamxy=-0.6', struct('lambda_IBVS',[-0.6;-0.6;-2.0;0;0;0])};
    S(end+1,:) = {5,'Cho lamxy=-1.2', struct('lambda_IBVS',[-1.2;-1.2;-2.0;0;0;0])};
    S(end+1,:) = {5,'Cho vsatxy=0.7', struct('v_sat',[0.7;0.7;0.7;0.2])};

    nspec = size(S,1);
    global CMP_OVERRIDE SWEEP_STATE %#ok<GVMIS>
    SWEEP_STATE = struct();
    SWEEP_STATE.S      = S;
    SWEEP_STATE.seeds  = seeds;
    SWEEP_STATE.trajs  = {"Static","Linear","Sinusoidal","Lissajous","Circular"};
    SWEEP_STATE.acc    = zeros(nspec,6);  % [sp land fovok sumxy_land sumvrel_land total]
    SWEEP_STATE.i      = 0;
    SWEEP_STATE.njobs  = nspec * 5 * numel(seeds);

    fprintf('\n==== baseline deep sweep | seeds=[%s] @ IC2 | %d runs/spec ====\n', ...
            num2str(seeds), 5*numel(seeds));

    while SWEEP_STATE.i < SWEEP_STATE.njobs
        global CMP_OVERRIDE SWEEP_STATE %#ok<GVMIS,REDEFGI>
        SWEEP_STATE.i = SWEEP_STATE.i + 1;
        [si,ti,sdi] = ind2sub([size(SWEEP_STATE.S,1),5,numel(SWEEP_STATE.seeds)], SWEEP_STATE.i);
        SWEEP_STATE.cur_si = si;   % survives the harness clearvars (si itself does not)
        CMP_OVERRIDE = SWEEP_STATE.S{si,3};
        CTRL_SEL  = SWEEP_STATE.S{si,1};   %#ok<NASGU>
        TRAJ_TYPE = SWEEP_STATE.trajs{ti}; %#ok<NASGU>
        MC_SEED   = SWEEP_STATE.seeds(sdi);%#ok<NASGU>

        visualControl_comparison;   % runs one sim; clears workspace except SWEEP_STATE

        si = SWEEP_STATE.cur_si;    % restore (cleared by the harness)
        a = SWEEP_STATE.acc(si,:);
        a(6) = a(6) + 1;
        if exist('fov_fail','var') && ~fov_fail, a(3) = a(3) + 1; end
        if exist('landed','var') && landed
            a(2) = a(2) + 1; a(4) = a(4) + xy_err; a(5) = a(5) + rel_vel;
            if precise && soft, a(1) = a(1) + 1; end
        end
        SWEEP_STATE.acc(si,:) = a;
    end

    % --- report -----------------------------------------------------------
    fprintf('\n%-22s  SP    LAND   FoV    meanXY  meanVrel\n','spec');
    N = 5*numel(SWEEP_STATE.seeds);
    for si = 1:size(SWEEP_STATE.S,1)
        a = SWEEP_STATE.acc(si,:);
        mxy = a(4)/max(a(2),1); mvr = a(5)/max(a(2),1);
        if SWEEP_STATE.S{si,1} ~= 2 && si>1 && SWEEP_STATE.S{si,1}~=SWEEP_STATE.S{si-1,1}
            fprintf('\n');
        end
        fprintf('%-22s %2d/%-2d  %2d/%-2d  %2d/%-2d   %5.3f   %5.3f\n', ...
            SWEEP_STATE.S{si,2}, a(1),N, a(2),N, a(3),N, mxy, mvr);
    end
    CMP_OVERRIDE = [];
    fprintf('\n==== baseline deep sweep complete ====\n');
end
