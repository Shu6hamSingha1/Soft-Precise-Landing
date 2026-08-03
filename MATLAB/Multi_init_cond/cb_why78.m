%% CB_WHY78  Why does BOTH (E.25 + pr-tight) fail at 7x/8x where ref (E.5) holds?
%   Run ref vs BOTH at 7x on the stress cells, find the distinguishing cell, and trace
%   the TERMINAL dynamics: switching term (au_sw), kappa, |sigma|, engR/engH, |a_u_xy|,
%   v_rel. Tight E delivers FULL kappa -> does the switching term detonate at the deck
%   under stress (the balloon mechanism) where ref's wider E keeps it linear?
%
% Run:  cd MATLAB/Multi_init_cond; cb_why78
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

base = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]),'Pleak',diag([0.5,0.5,1.5]));
REF  = setfield(base,'E',diag([0.5,0.5,0.5])); %#ok<*SFLD>
BOTH = setfield(setfield(setfield(base,'E',diag([0.25,0.25,0.5])),'Xi_r',diag([0.3,0.3])),'p_rinf',[0.85;0.85]);
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
cells = { 'Sinusoidal',2; 'Sinusoidal',4; 'Sinusoidal',5; 'Circular',3; 'Circular',5 };

% find a 7x cell where REF holds but BOTH fails
fprintf('\n=== 7x per-cell: REF vs BOTH ===\n');
target=0;
for c=1:size(cells,1)
    rr=run1(REF, 7,p0(cells{c,2},:),cells{c,1});
    rb=run1(BOTH,7,p0(cells{c,2},:),cells{c,1});
    fprintf('  %-11s IC%d | REF SP=%d xy=%.3f | BOTH SP=%d xy=%.3f\n', ...
        cells{c,1},cells{c,2}, rr.sp,rr.xy, rb.sp,rb.xy);
    if rr.sp && ~rb.sp && target==0, target=c; end
end
if target==0, fprintf('\n(no distinguishing cell at 7x; tracing Sinusoidal IC2 at 8x)\n'); target=1; sc=8; else, sc=7; end

tc=cells{target,1}; ti=cells{target,2};
fprintf('\n=== TERMINAL TRACE: %s IC%d at %dx ===\n', tc, ti, sc);
for cf={{'REF E.5',REF},{'BOTH E.25+pr',BOTH}}
    nm=cf{1}{1}; d=run1full(cf{1}{2},sc,p0(ti,:),tc);
    fprintf('  --- %-14s land=%d xy=%.3f ---\n', nm, d.sp, d.xy);
    fprintf('     t    z   | |h_e| engR engH | kap_xy |sig_xy| | REACH SWITCH | |au_xy|  v_rel\n');
    for zt=[2.0 1.5 1.0 0.7 0.5 0.35 0.25 0.21]      % sample by ALTITUDE (terminal descent)
        j=find(d.z<=zt,1);                            % first time below this altitude
        if isempty(j), continue; end
        fprintf('   %5.1f %4.2f | %5.2f %4.2f %4.2f | %6.2f %6.2f | %5.1f %6.1f | %6.1f %6.3f\n', ...
            d.t(j),d.z(j),d.he(j),d.engR(j),d.engH(j),d.kx(j),d.sx(j),d.reach(j),d.switch(j),d.au(j),d.v(j));
    end
    fprintf('     LAST: t=%.1f z=%.2f |h_e|=%.2f engR=%.2f kap=%.2f sig=%.2f SWITCH=%.1f au=%.1f v=%.3f\n', ...
        d.t(end),d.z(end),d.he(end),d.engR(end),d.kx(end),d.sx(end),d.switch(end),d.au(end),d.v(end));
end
clear global VDF_OVERRIDE STRESS_SCALE

function r = run1(cfg,s,p0row,traj)
    global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>
    VDF_OVERRIDE=cfg; STRESS_SCALE=s;
    rr=run_simulation([p0row(:);1;0;0;0;zeros(6,1)],traj,[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
    r.sp=rr.precise&&rr.soft; r.xy=rr.final_xy;
end
function d = run1full(cfg,s,p0row,traj)
    global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>
    VDF_OVERRIDE=cfg; STRESS_SCALE=s;
    r=run_simulation([p0row(:);1;0;0;0;zeros(6,1)],traj,[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
    o=r.data; idx=o.idx; d.t=o.tRange(1:idx); d.t=d.t(:)'; d.sp=r.precise&&r.soft; d.xy=r.final_xy;
    d.z=1./max(o.beta_log(1:idx),1e-6);
    d.he=vecnorm(o.V_h_e(1:2,1:idx));
    rbe=abs(o.s_e_log(:,1:idx)./o.P.phi_max(:)); d.engR=max(rbe./max(o.p_r_log(:,1:idx),eps),[],1);
    d.engH=max(abs(o.V_h_e(1:2,1:idx))./max(o.p_h_log(1:2,1:idx),eps),[],1);
    d.kx=max(o.kappa_log(1:2,1:idx),[],1); d.sx=max(abs(o.sigma(1:2,1:idx)),[],1);
    d.reach=o.au_comp_log(1,1:idx); d.switch=o.au_comp_log(2,1:idx);
    d.au=vecnorm(o.I_a_cd(1:2,1:idx)); d.v=vecnorm(o.X_DS(8:9,1:idx)-o.dx_t(1:2,1:idx));
end
