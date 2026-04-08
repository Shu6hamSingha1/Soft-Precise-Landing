%% Speed-envelope check for kappa_0-only candidate
% Baseline envelope (from sweep_speed.mat):
%   Linear 1.25, Sin 1.75, Circ 1.00, Liss 1.50
% Test whether kappa_0 = [0.125;0.125;0.25] alone preserves this envelope.

clc; clear;

trajList = ["Linear","Sinusoidal","Circular","Lissajous"];
p0 = [0,0,-5; 0,0,-7; 0,0,-3; 2,2,-5; -2,-2,-5];
numRuns = size(p0,1);

ovr_kappa = struct('kappa_0', [0.125; 0.125; 0.25]);

mults = [1.0, 1.25, 1.5, 1.75, 2.0];

results = struct('traj',{},'mult',{},'n_land',{},'mean_t',{},'max_xy',{},'fails',{});
row = 0;

for t = 1:numel(trajList)
    trajType = trajList(t);
    fprintf('\n--- %s ---\n', trajType);
    for m = mults
        row = row + 1;
        [nl,mt,mxy,maxxy,failIC] = run_5ic_speed(p0,numRuns,trajType,m,ovr_kappa);
        results(row).traj   = trajType;
        results(row).mult   = m;
        results(row).n_land = nl;
        results(row).mean_t = mt;
        results(row).max_xy = maxxy;
        results(row).fails  = failIC;
        fprintf('  mult=%.2f  land=%d/5  mean_t=%.2f  max_xy=%.4f  fails=[%s]\n', ...
                m,nl,mt,maxxy,num2str(failIC));
        if nl < numRuns
            fprintf('  -> first failure, stopping %s\n', trajType);
            break;
        end
    end
end

save(fullfile(fileparts(mfilename('fullpath')),'Datasets','validate_kappa0_speed.mat'), ...
     'results','mults','trajList');

fprintf('\n======== KAPPA_0-ONLY SPEED ENVELOPE ========\n');
for t = 1:numel(trajList)
    lastPass = NaN; firstFail = NaN;
    for r = 1:numel(results)
        if results(r).traj ~= trajList(t), continue; end
        if results(r).n_land == numRuns, lastPass = results(r).mult; end
        if results(r).n_land < numRuns && isnan(firstFail)
            firstFail = results(r).mult;
        end
    end
    fprintf('  %-12s  last 5/5: x%.2f   first fail: x%s\n', ...
            trajList(t), lastPass, num2str(firstFail));
end

% =====================================================================
function [n_land,mean_t,mean_xy,max_xy,fail_ICs] = run_5ic_speed(p0,numRuns,trajType,speed_mult,ovr)
    landed=false(numRuns,1); t_l=nan(numRuns,1); xy_f=nan(numRuns,1);
    for k=1:numRuns
        q0=[1;0;0;0]; v0=zeros(3,1); w0=zeros(3,1);
        x0=[p0(k,:)'; q0; v0; w0];
        rng(1000+k);
        try
            tmp = run_simulation(x0,trajType,ovr,speed_mult);
        catch ME
            fprintf('  Run %d ERR: %s\n',k,ME.message); continue;
        end
        if isfield(tmp,'success'),  landed(k)=tmp.success;  end
        if isfield(tmp,'final_t'),  t_l(k)=tmp.final_t;     end
        if isfield(tmp,'final_xy'), xy_f(k)=tmp.final_xy;   end
    end
    n_land=sum(landed);
    mean_t=mean(t_l(landed),'omitnan');
    mean_xy=mean(xy_f(landed),'omitnan');
    max_xy=max(xy_f(landed),[],'omitnan');
    fail_ICs=find(~landed)';
    if isempty(mean_t)||isnan(mean_t), mean_t=NaN; end
    if isempty(mean_xy)||isnan(mean_xy), mean_xy=NaN; end
    if isempty(max_xy)||isnan(max_xy), max_xy=NaN; end
end
