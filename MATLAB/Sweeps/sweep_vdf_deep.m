function sweep_vdf_deep(seeds)
%SWEEP_VDF_DEEP  Fresh one-at-a-time deep sweep of the baked VDF-ASMC gains.
%   sweep_vdf_deep(seeds) probes each high-leverage vdf_params lever around its
%   baked value, scoring soft-precise (SP) over the canonical 5x5 realistic set
%   (5 trajs x 5 ICs) across a SEED ENSEMBLE, plus the Sinusoidal lambda=1.4
%   +40% edge. Baked value is marked [*]. A good candidate keeps SP at the
%   ensemble ceiling AND holds the edge. Uses the VDF_OVERRIDE hook.
%
%   seeds : vector of noise seeds (default [1 2 3 4 5]).

    if nargin < 1 || isempty(seeds), seeds = 1:5; end
    this_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(this_dir,'..','Common'));
    addpath(fullfile(this_dir,'..','VDF_ASMC'));
    addpath(fullfile(this_dir,'..','Multi_init_cond'));

    trajs = {"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs   = {[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    cfg   = struct('NOISE',1,'GE',1,'delay',1);
    nS = numel(seeds); tot = 25*nS;

    % --- Deep-sweep grid: {name, {values...}, baked-index} -----------------
    g = { ...
      'chi_r',  {[0.85;0.85],[1.0;1.0],[1.15;1.15],[1.30;1.30],[1.45;1.45]}, 3; ...
      'chi_z',  {0.05,0.075,0.1,0.15,0.2}, 3; ...
      'Gamma',  {diag([0.35,0.40,0.75]),diag([0.4375,0.5,0.75]),diag([0.55,0.60,0.75]),diag([0.70,0.70,0.75])}, 2; ...
      'E',      {diag([1,1,0.40]),diag([1,1,0.5]),diag([1,1,0.65]),diag([1,1,0.80])}, 2; ...
      'N',      {diag([0.01,0.01,0.01]),diag([0.02,0.02,0.02]),diag([0.04,0.04,0.04])}, 2; ...
      'p_hinf', {[0.35;0.35;1.5],[0.5;0.5;1.5],[0.70;0.70;1.5],[0.5;0.5;1.2],[0.5;0.5;1.8]}, 2; ...
      'kR',     {diag([2.0,1.5,0.5]),diag([2.5,1.5,0.5]),diag([3.0,1.5,0.5])}, 2; ...
      'kOmega', {diag([0.3,0.3,0.15]),diag([0.3,0.3,0.2]),diag([0.3,0.3,0.3])}, 2; ...
      'h_rd',   {-0.46,-0.42,-0.38}, 2; ...
      'theta_cap', {deg2rad(50),deg2rad(60),deg2rad(70)}, 2; ...
      'Pleak',  {diag([1.0,1.0,5.0]),diag([1.5,1.5,5.0]),diag([2.5,2.5,5.0])}, 2; ...
    };

    global VDF_OVERRIDE
    fprintf('\n==== VDF-ASMC deep sweep | seeds=[%s] | %d-run ensemble per point ====\n', ...
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
            % edge: Sin lambda=1.4 from origin, across the same seeds
            esp = 0;
            for sdi = 1:nS
                o = run_simulation([0;0;-5;1;0;0;0;zeros(6,1)],"Sinusoidal",[],1.4,cfg,seeds(sdi));
                esp = esp + (o.success && o.precise && o.soft);
            end
            mark = ''; if vi==baked, mark=' [*baked]'; end
            ff = topfails(fc);
            fprintf('   %-26s SP=%3d/%-3d  edge=%d/%d  %s%s\n', valstr(name,vals{vi}), sp, tot, esp, nS, ff, mark);
        end
    end
    VDF_OVERRIDE = [];
    fprintf('\n==== deep sweep complete ====\n');
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
