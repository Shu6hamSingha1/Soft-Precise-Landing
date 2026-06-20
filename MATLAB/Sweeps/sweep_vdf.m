function results = sweep_vdf(param, values, varargin)
%SWEEP_VDF  Clean combined-barrier gain sweep via the vdf_params VDF_OVERRIDE hook.
%   results = sweep_vdf(param, values) sweeps the single vdf_params field <param>
%   over the cell array <values>, running the VERIFIED controller (run_simulation,
%   which uses the VDF_ASMC blocks) and reporting soft-precise outcomes on:
%     - the canonical 5x5 realistic set (the locked 25/25 no-regression gate), and
%     - the marginal Sinusoidal lambda=1.4 case (seeds 1-2), the +40% edge.
%   A good candidate KEEPS 25/25 AND holds/fixes Sin-1.4. Replaces the retired
%   K_override gain harnesses (sweep_deep/plasmc/plasmc_inner), which targeted the
%   old back-mapped controller (rp/ri/rd SEN-PID + rho_fov/l_fov cone clamp -- all
%   removed). Sweep ANY combined-barrier field by its vdf_params name.
%
%   Name/value options:
%     'seed'    fixed validation noise seed for the 25-set (default 1)
%     'edge'    test the Sin lambda=1.4 +40% edge (default true)
%
%   Examples:
%     sweep_vdf('chi_r',  {[0.85;0.85],[1.0;1.0],[1.15;1.15]})
%     sweep_vdf('Gamma',  {diag([0.4375,0.5,0.75]), diag([0.7,0.7,0.75])})
%     sweep_vdf('p_hinf', {[0.5;0.5;1.5],[0.35;0.35;1.5]})
%     sweep_vdf('theta_cap', {deg2rad(45), deg2rad(60), deg2rad(75)})

    this_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(this_dir, '..', 'Common'));
    addpath(fullfile(this_dir, '..', 'VDF_ASMC'));
    addpath(fullfile(this_dir, '..', 'Multi_init_cond'));   % run_simulation

    p = inputParser;
    addParameter(p, 'seed', 1);
    addParameter(p, 'edge', true);
    parse(p, varargin{:});
    seed = p.Results.seed;  do_edge = p.Results.edge;

    trajs = {"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs   = {[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    cfg   = struct('NOISE',1,'GE',1,'delay',1);

    global VDF_OVERRIDE
    results = struct('value',{},'sp25',{},'fails',{},'edge_sp',{});
    fprintf('  sweep_vdf: %s  (gate: 25/25 realistic seed %d', param, seed);
    if do_edge, fprintf(' + Sin lambda=1.4 seeds 1-2'); end
    fprintf(')\n');

    for vi = 1:numel(values)
        val = values{vi};
        if vi == 1 && isBaseline(val)
            VDF_OVERRIDE = [];            % baseline row (baked values)
        else
            VDF_OVERRIDE = struct(param, val);
        end

        sp = 0; fails = {};
        for ti = 1:5
            for ii = 1:5
                x0 = [ICs{ii}(:); 1;0;0;0; zeros(6,1)];
                o = run_simulation(x0, trajs{ti}, [], 1.0, cfg, seed);
                if o.success && o.precise && o.soft
                    sp = sp + 1;
                else
                    fails{end+1} = sprintf('%s%d', trajs{ti}{1}(1), ii); %#ok<AGROW>
                end
            end
        end

        edge_sp = NaN;
        if do_edge
            edge_sp = 0;
            for sd = 1:2
                o = run_simulation([0;0;-5;1;0;0;0;zeros(6,1)], "Sinusoidal", [], 1.4, cfg, sd);
                edge_sp = edge_sp + (o.success && o.precise && o.soft);
            end
        end

        flag = ''; if sp == 25 && (~do_edge || edge_sp == 2), flag = '  <== keeps 25/25 + edge'; end
        fprintf('  %-14s : 25set=%2d/25  Sin1.4=%s  %s%s\n', ...
                valstr(val), sp, edgestr(edge_sp), strjoin(fails, ','), flag);
        results(end+1) = struct('value',val,'sp25',sp,'fails',{fails},'edge_sp',edge_sp); %#ok<AGROW>
    end
    VDF_OVERRIDE = [];   % restore baked controller
end

function b = isBaseline(v)
    b = ischar(v) || (isstring(v) && strcmpi(v,"baseline"));
end
function s = valstr(v)
    if isBaseline(v), s = 'baseline'; return; end
    s = mat2str(v(:)', 4);
end
function s = edgestr(e)
    if isnan(e), s = '--'; else, s = sprintf('%d/2', e); end
end
