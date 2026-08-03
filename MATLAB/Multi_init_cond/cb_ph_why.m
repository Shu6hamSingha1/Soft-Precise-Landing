%% CB_PH_WHY  Why does tightening p_h overtake but tightening p_r does NOT?
%   Trace the p_h-tightened case (locked base + Xi_h.35) at 7x, showing BOTH funnels:
%     position:  r_bar_e, p_r, engR=|r_bar_e|/p_r   (the control NULLS position)
%     flow:      h_e,     p_h, engH=|h_e|/p_h,  v_rel (h_e=v_rel/z)
%   Hypothesis: position CAN be nulled (r_bar_e->small, p_r tightenable), but at the deck
%   a residual v_rel makes h_e=v_rel/z grow (1/z) -> h_e is IRREDUCIBLE -> the contracting
%   p_h meets it -> breach. So p_h is un-tightenable because its quantity (v/z) can't be
%   driven to 0 at the deck, while p_r's quantity (position) can.
%
% Run:  cd MATLAB/Multi_init_cond; cb_ph_why
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

LK = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]), ...
            'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]),'Xi_r',diag([0.3,0.3]),'p_rinf',[0.85;0.85]);
PHT = setfield(LK,'Xi_h',diag([0.35,0.35,0.35])); %#ok<*SFLD>  % p_h tightened
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
cells = { 'Sinusoidal',2; 'Sinusoidal',4; 'Sinusoidal',5; 'Circular',3; 'Circular',5 };

% find a 7x cell where PHT fails
target=1;
for c=1:size(cells,1)
    if ~run1(PHT,7,p0(cells{c,2},:),cells{c,1}).sp, target=c; break; end
end
tc=cells{target,1}; ti=cells{target,2};
fprintf('\n=== p_h-tight (Xi_h.35) %s IC%d @7x ===\n', tc, ti);
d=full(PHT,7,p0(ti,:),tc);
fprintf('  land=%d xy=%.3f\n', d.sp, d.xy);
fprintf('    z   | r_bar_e  p_r  engR | h_e    p_h   engH | v_rel  switch\n');
for zt=[2.0 1.5 1.0 0.7 0.5 0.35 0.25 0.21]
    j=find(d.z<=zt,1); if isempty(j),continue;end
    fprintf('  %4.2f |  %5.3f  %4.2f %5.2f | %5.3f %5.2f %5.2f | %5.3f %5.1f\n', ...
        d.z(j), d.rbe(j),d.pr(j),d.engR(j), d.he(j),d.ph(j),d.engH(j), d.v(j),d.sw(j));
end
clear global VDF_OVERRIDE STRESS_SCALE
fprintf('\n(expect: engR stays LOW (position nulled, p_r held) but engH BREACHES at deck\n as p_h contracts into the residual-v_rel-driven h_e=v_rel/z)\n');

function r = run1(cfg,s,p0row,traj)
    global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>
    VDF_OVERRIDE=cfg; STRESS_SCALE=s;
    rr=run_simulation([p0row(:);1;0;0;0;zeros(6,1)],traj,[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
    r.sp=rr.precise&&rr.soft;
end
function d = full(cfg,s,p0row,traj)
    global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>
    VDF_OVERRIDE=cfg; STRESS_SCALE=s;
    r=run_simulation([p0row(:);1;0;0;0;zeros(6,1)],traj,[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
    o=r.data; idx=o.idx; d.z=1./max(o.beta_log(1:idx),1e-6); d.sp=r.precise&&r.soft; d.xy=r.final_xy;
    rbe=abs(o.s_e_log(:,1:idx)./o.P.phi_max(:)); [d.rbe,ax]=max(rbe,[],1);
    d.pr=o.p_r_log(1,1:idx); d.engR=max(rbe./max(o.p_r_log(:,1:idx),eps),[],1);
    d.he=vecnorm(o.V_h_e(1:2,1:idx)); d.ph=o.p_h_log(1,1:idx);
    d.engH=max(abs(o.V_h_e(1:2,1:idx))./max(o.p_h_log(1:2,1:idx),eps),[],1);
    d.v=vecnorm(o.X_DS(8:9,1:idx)-o.dx_t(1:2,1:idx)); d.sw=o.au_comp_log(2,1:idx);
end
