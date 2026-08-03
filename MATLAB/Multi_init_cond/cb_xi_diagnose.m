%% CB_XI_DIAGNOSE  WHY does faster Xi fail? Trace the time-series: does the funnel
%   reach its floor BEFORE the error converges -> engagement spikes -> kappa ramps
%   -> a_u spikes -> breach? Compare baked Xi (X0) vs fast Xi (X3) on offset cells.
%
%   Funnel p(t)=e^{-Xi t}(p0-pinf)+pinf. Floors pinned BAKED (p_hinf [1;1;1.5],
%   p_rinf [1;1]); ONLY Xi differs. The "race": funnel-contraction time 1/Xi vs
%   error-convergence time. If funnel reaches floor while error still large -> breach.
%
% Run:  cd MATLAB/Multi_init_cond; cb_xi_diagnose
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE %#ok<GVMIS>

C = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05], ...
           'N',diag([0.10,0.10,0.10]),'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]), ...
           'p_hinf',[1.0;1.0;1.5],'p_rinf',[1.0;1.0]);
mk = @(xih,xir) setfield(setfield(C,'Xi_h',diag([xih xih xih])),'Xi_r',diag([xir xir])); %#ok<SFLD>
X0 = mk(0.2,0.1);  X3 = mk(0.8,0.7);
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
cells = { 'Sinusoidal',2; 'Lissajous',3; 'Circular',3 };

for c=1:size(cells,1)
    traj=cells{c,1}; ic=cells{c,2};
    fprintf('\n############ %s IC%d ############\n', traj, ic);
    for cf = {{'X0 baked Xi.2/.1',X0},{'X3 fast Xi.8/.7',X3}}
        nm=cf{1}{1}; d=run(cf{1}{2}, p0(ic,:), traj);
        report(nm, d);
    end
end

function d = run(cfgOv, p0row, traj)
    global VDF_OVERRIDE %#ok<GVMIS>
    VDF_OVERRIDE = cfgOv;
    x0 = [p0row(:); 1;0;0;0; zeros(3,1); zeros(3,1)];
    r = run_simulation(x0, traj, [], 1.0, struct('NOISE',1,'GE',1,'delay',1), 1);
    d = r.data; d.landed = r.success; d.fail_xy = r.final_xy;
end

function report(nm, d)
    idx=d.idx; t=d.tRange(1:idx); t=t(:)';
    z = 1./max(d.beta_log(1:idx),1e-6);                 % above-target gap (m)
    pr = d.p_r_log(:,1:idx);  ph = d.p_h_log(:,1:idx);
    rbe = abs(d.s_e_log(:,1:idx)./d.P.phi_max(:));      % |r_bar_e| (2xN)
    engR = max(rbe./max(pr,eps),[],1);                  % position engagement
    engHz= abs(d.V_h_e(3,1:idx))./max(ph(3,1:idx),eps); % z optic-flow engagement
    kx  = max(d.kappa_log(1:2,1:idx),[],1);             % lateral kappa
    sx  = max(abs(d.sigma(1:2,1:idx)),[],1);            % |sigma_xy|
    aux = vecnorm(d.I_a_cd(1:2,1:idx));                 % |a_u_xy|

    % timeline markers
    prfloor = mean(pr,1);  % position funnel mean width
    i_floor = find(prfloor < 1.05*mean(d.P.p_rinf), 1);     % funnel reaches ~floor
    i_conv  = find(max(rbe,[],1) < 0.30, 1);                 % error "converged"
    [meR,iR]=max(engR); [mk,ik]=max(kx); [ma,ia]=max(aux);
    g=@(i) iif(~isempty(i), sprintf('t=%.1f z=%.2f', t(min(i,idx)), z(min(i,idx))), 'never');

    fprintf('  --- %-18s landed=%d final_xy=%.3f ---\n', nm, d.landed, d.fail_xy);
    fprintf('     funnel->floor: %s | err conver(|rbe|<.3): %s  => %s\n', ...
        g(i_floor), g(i_conv), raceword(i_floor,i_conv));
    fprintf('     peak engR=%.2f @ %s | peak kappa_xy=%.2f @ %s | peak|a_u_xy|=%.1f @ %s\n', ...
        meR, g(iR), mk, g(ik), ma, g(ia));
    % coarse timeline every ~1.5 s
    fprintf('       t    z    p_r  |rbe| engR  k_xy |s_xy| |au_xy|\n');
    for tt = 0:1.5:t(end)
        [~,j]=min(abs(t-tt));
        fprintf('     %4.1f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f %6.1f\n', ...
            t(j), z(j), prfloor(j), max(rbe(:,j)), engR(j), kx(j), sx(j), aux(j));
    end
end
function s=raceword(a,b), if isempty(b)&&~isempty(a), s='FLOOR-FIRST (err never converged!)';
    elseif isempty(a), s='floor-never'; elseif a<b, s='*** FLOOR REACHED BEFORE CONVERGENCE ***';
    else s='converged-first (OK)'; end, end
function o=iif(c,a,b), if c,o=a; else,o=b; end, end
