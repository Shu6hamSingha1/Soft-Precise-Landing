function sweep_vdf_deep2(seeds)
%SWEEP_VDF_DEEP2  Part 2 of the fresh deep sweep: the manuscript control levers
%   NOT covered by sweep_vdf_deep (the funnel SHAPES, switching-gain init,
%   saturation guard, the full Yaw ASMC, and the accel LPF). Same 125-run seed
%   ensemble + Sin lambda=1.4 edge, via the VDF_OVERRIDE hook. Together the two
%   scripts cover every sweepable lever in manuscript Table sup:control params.
%   (phi_max is hardware-derived per the paper; the CBF drift look-ahead tau is
%   internal to the cbf block, not a vdf_params field -> not swept here.)
%
%   seeds : noise-seed ensemble (default 1:5).

    if nargin < 1 || isempty(seeds), seeds = 1:5; end
    this_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(this_dir,'..','Common'));
    addpath(fullfile(this_dir,'..','VDF_ASMC'));
    addpath(fullfile(this_dir,'..','Multi_init_cond'));

    trajs = {"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs   = {[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    cfg   = struct('NOISE',1,'GE',1,'delay',1);
    nS = numel(seeds); tot = 25*nS;

    g = { ...
      % --- Image-Feature Funnel shape ---
      'Xi_r',   {diag([0.05,0.05]),diag([0.10,0.10]),diag([0.20,0.20])}, 2; ...
      'p_r0',   {[1.1;1.1],[1.2;1.2],[1.5;1.5]}, 2; ...
      'p_rinf', {[1.0;1.0],[1.1;1.1],[1.2;1.2]}, 1; ...   % Standing Cond 1: >=1
      % --- Optical-Flow Funnel shape ---
      'Xi_h',   {diag([0.1,0.1,0.1]),diag([0.2,0.2,0.2]),diag([0.35,0.35,0.35])}, 2; ...
      'p_h0',   {[20;20;4],[25;25;4],[30;30;5]}, 2; ...
      'S_margin', {0.02,0.05,0.10}, 2; ...
      % --- Optic-Flow ASMC switching-gain init ---
      'kappa0', {[0.0625;0.0625;0.125],[0.125;0.125;0.25],[0.25;0.25;0.5]}, 2; ...
      % --- Yaw ASMC (full 6 levers) ---
      'Omega_a',  {0.25,0.5,1.0}, 2; ...
      'Gamma_a',  {0.25,0.5,1.0}, 2; ...
      'n_a',      {0.5,1.0,2.0}, 2; ...
      'p_a',      {1.0,2.0,4.0}, 2; ...
      'kappa_a0', {1.0,2.0,4.0}, 2; ...
      'E_a',      {1.5,3.0,6.0}, 2; ...
      % --- Accel LPF (CBF upstream) ---
      'tau_ia',   {0.04,0.08,0.15}, 2; ...
    };

    global VDF_OVERRIDE
    fprintf('\n==== VDF-ASMC deep sweep PART 2 | seeds=[%s] | %d-run ensemble ====\n', ...
            num2str(seeds), tot);

    for k = 1:size(g,1)
        name = g{k,1}; vals = g{k,2}; baked = g{k,3};
        fprintf('\n-- %s --\n', name);
        for vi = 1:numel(vals)
            if vi == baked, VDF_OVERRIDE = []; else, VDF_OVERRIDE = struct(name, vals{vi}); end
            sp = 0; fc = containers.Map('KeyType','char','ValueType','double');
            for sdi = 1:nS
                for ti = 1:5
                    for ii = 1:5
                        x0 = [ICs{ii}(:); 1;0;0;0; zeros(6,1)];
                        o = run_simulation(x0, trajs{ti}, [], 1.0, cfg, seeds(sdi));
                        if o.success && o.precise && o.soft
                            sp = sp + 1;
                        else
                            key = sprintf('%s%d', trajs{ti}{1}(1), ii);
                            if isKey(fc,key), fc(key)=fc(key)+1; else, fc(key)=1; end
                        end
                    end
                end
            end
            esp = 0;
            for sdi = 1:nS
                o = run_simulation([0;0;-5;1;0;0;0;zeros(6,1)],"Sinusoidal",[],1.4,cfg,seeds(sdi));
                esp = esp + (o.success && o.precise && o.soft);
            end
            mark = ''; if vi==baked, mark=' [*baked]'; end
            fprintf('   %-26s SP=%3d/%-3d  edge=%d/%d  %s%s\n', valstr(name,vals{vi}), sp, tot, esp, nS, topfails(fc), mark);
        end
    end
    VDF_OVERRIDE = [];
    fprintf('\n==== deep sweep PART 2 complete ====\n');
end

function s = valstr(name,v)
    if isscalar(v), s = sprintf('%s=%.4g', name, v);
    elseif isvector(v), s = sprintf('%s=[%s]', name, num2str(v(:)',4));
    else, s = sprintf('%s=diag[%s]', name, num2str(diag(v)',4)); end
end
function s = topfails(fc)
    if fc.Count==0, s='(clean)'; return; end
    ks = keys(fc); vs = cell2mat(values(fc));
    [vs,ix] = sort(vs,'descend'); ks = ks(ix);
    parts = cell(1,numel(ks));
    for i=1:numel(ks), parts{i}=sprintf('%s:%d',ks{i},vs(i)); end
    s = strjoin(parts, ' ');
end
