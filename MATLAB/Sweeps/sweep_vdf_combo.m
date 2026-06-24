function sweep_vdf_combo(seeds)
%SWEEP_VDF_COMBO  Validate stacked deep-sweep winners vs the baked config.
%   Tests candidate combinations of the levers that individually beat the baked
%   114/125 in the fresh deep sweep, on a LARGER seed ensemble (default 1:10)
%   to separate real gains from per-cell noise (+-1 cell = +-0.8%). Reports the
%   baked baseline plus each candidate; "SAFE" stacks avoid the levers that were
%   baked for deterministic limit-cycle reasons (kR, kOmega_z, E_z).
%
%   seeds : noise-seed ensemble (default 1:10).

    if nargin < 1 || isempty(seeds), seeds = 1:10; end
    this_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(this_dir,'..','Common'));
    addpath(fullfile(this_dir,'..','VDF_ASMC'));
    addpath(fullfile(this_dir,'..','Multi_init_cond'));

    trajs = {"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs   = {[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    cfg   = struct('NOISE',1,'GE',1,'delay',1);
    nS = numel(seeds); tot = 25*nS;

    % candidate list: {label, override-struct or []}
    C = { ...
      'baked', []; ...
      'h_rd=-0.38', struct('h_rd',-0.38); ...
      'yaw0.25', struct('Omega_a',0.25,'Gamma_a',0.25); ...
      'kappa0/2', struct('kappa0',[0.0625;0.0625;0.125]); ...
      'SAFE: hrd+yaw+k0', struct('h_rd',-0.38,'Omega_a',0.25,'Gamma_a',0.25,'kappa0',[0.0625;0.0625;0.125]); ...
      'SAFE+chi_z.075', struct('h_rd',-0.38,'Omega_a',0.25,'Gamma_a',0.25,'kappa0',[0.0625;0.0625;0.125],'chi_z',0.075); ...
      'AGGR(+kR3,kOz.3,Ez.65)', struct('h_rd',-0.38,'Omega_a',0.25,'Gamma_a',0.25,'kappa0',[0.0625;0.0625;0.125], ...
                                       'kR',diag([3,1.5,0.5]),'kOmega',diag([0.3,0.3,0.3]),'E',diag([1,1,0.65])); ...
    };

    global VDF_OVERRIDE
    fprintf('\n==== VDF-ASMC combo validation | seeds=[%s] | %d-run ensemble ====\n', ...
            num2str(seeds(1)) , tot); fprintf('   (seeds %d..%d)\n', seeds(1), seeds(end));

    for ci = 1:size(C,1)
        VDF_OVERRIDE = C{ci,2};
        sp = 0; fc = containers.Map('KeyType','char','ValueType','double');
        for sdi = 1:nS
            for ti = 1:5
                for ii = 1:5
                    x0 = [ICs{ii}(:); 1;0;0;0; zeros(6,1)];
                    o = run_simulation(x0, trajs{ti}, [], 1.0, cfg, seeds(sdi));
                    if o.success && o.precise && o.soft, sp = sp + 1;
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
        fprintf('   %-26s SP=%3d/%-3d (%5.1f%%)  edge=%d/%d  %s\n', ...
                C{ci,1}, sp, tot, 100*sp/tot, esp, nS, topfails(fc));
    end
    VDF_OVERRIDE = [];
    fprintf('\n==== combo validation complete ====\n');
end

function s = topfails(fc)
    if fc.Count==0, s='(clean)'; return; end
    ks = keys(fc); vs = cell2mat(values(fc));
    [vs,ix] = sort(vs,'descend'); ks = ks(ix);
    parts = cell(1,min(numel(ks),10));
    for i=1:numel(parts), parts{i}=sprintf('%s:%d',ks{i},vs(i)); end
    s = strjoin(parts, ' ');
end
