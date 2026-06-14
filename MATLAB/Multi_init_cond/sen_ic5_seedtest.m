% IC5 [2,2,-3] noisy — matched-seed A/B of the SEN_FUNNEL demand-starvation fix.
% For each RNG seed, run the SAME noise realization under each config so the
% containment fix is judged deterministically (not against stochastic variance).
%
% Configs: baseline (legacy soft-clip) vs contain (SEN_CONTAIN_MODE=1: expand the
% effective funnel on breach so inward demand scales with |s_e_n| instead of
% collapsing).  Reports per-seed outcome + the saturation diagnostics, then a
% tally (soft+precise, fly-aways, FoV-fails) per config.
%
%   SEN_NSEEDS (env, default 12) controls how many seeds.   run sen_ic5_seedtest

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|RNG_SEED_OVERRIDE|SEN_.*|WS).*$';
close all; clc;

global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE WS ...
       SEN_PS0_OVERRIDE SEN_PSINF_OVERRIDE SEN_GAMMAS_OVERRIDE ...
       SEN_IZETASMAX_OVERRIDE SEN_CONTAIN_MODE SEN_RP_OVERRIDE ...
       SEN_RI_OVERRIDE SEN_RD_OVERRIDE THETA_CAP_OVERRIDE FOV_INSET_OVERRIDE ...
       GAMMA_XY_OVERRIDE E_XY_OVERRIDE P_XY_OVERRIDE KAPPA0_XY_OVERRIDE N_XY_OVERRIDE ...
       H_RD_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE SREF_GAMMA DH_D_CAP DH_D_TAU ...
       CBF_NO_REVERSE DIR_INSET_RELAX DRIFT_RECENTER DRIFT_GATED;

WS = struct();
WS.ic     = [2; 2; -3];
WS.nseeds = getenv_num('SEN_NSEEDS', 12);
% Outer-gain (K_rp/ri/rd) retune candidates vs baseline (rp9 ri0.1 rd1.4375).
% Seed 6 leaves Y uncorrected pre-breach (t<1.59) then winds izeta to the clamp;
% test whether faster P / more D-lead / less I-windup keeps Y inside the funnel.
% BREACH VALIDATION: aim = keep s_e_n inside p_s (no breach). Compare baseline,
% the no-regression command-cleaner (cap20), and the containment mode (which
% forces no-breach by EXPANDING p_s on contact -- tests whether "no breach by
% moving the boundary" buys a landing, vs genuine in-funnel tracking).
% DIRECTIONAL INSET: relax the box toward the FoV edge (px) ONLY on the step
% where the tight inset would REVERSE the demanded lateral sign (deliver the
% re-centering -Y instead of stripping it). Self-targeting -> good seeds keep
% the inset. dirinset=15 relaxes fully to the FoV edge; 10/8 = partial.
% DRIFT_RECENTER: seed the CBF's drift with the expected -g*cr2 re-centering, so
% it anticipates the -Y translation and stops stripping the lean. g in 1/s.
% STRIP-RATIO-GATED DRIFT (driftg = gain): re-centering anticipation scaled by
% how much -Y the tight CBF stripped -> self-targets the seed-6 flip, ~0 on the
% good seeds whose -Y survives.
WS.cfgs = { ...
  struct('name','base'), ...
  struct('name','dg0.5','driftg',0.5), ...
  struct('name','dg1.0','driftg',1.0), ...
  struct('name','dg2.0','driftg',2.0), ...
  struct('name','dg1.0cap','driftg',1.0,'dhcap',20), ...
};
WS.R = {};

for s = 1:WS.nseeds
  for ci = 1:numel(WS.cfgs)
    % RE-DECLARE globals each iteration: the canonical script's top-of-file
    % `clear` drops the global link for any global NOT re-declared by the
    % controller itself (the cbf2-only ones: CBF_NO_REVERSE/DIR_INSET_RELAX/
    % DRIFT_RECENTER). Without this they silently become locals from iter 2 on
    % and never reach cbf2_filter. (Found 2026-06-14: drift/norev/dir were no-ops.)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE WS ...
           SEN_PS0_OVERRIDE SEN_PSINF_OVERRIDE SEN_GAMMAS_OVERRIDE ...
           SEN_IZETASMAX_OVERRIDE SEN_CONTAIN_MODE SEN_RP_OVERRIDE ...
           SEN_RI_OVERRIDE SEN_RD_OVERRIDE THETA_CAP_OVERRIDE FOV_INSET_OVERRIDE ...
           GAMMA_XY_OVERRIDE E_XY_OVERRIDE P_XY_OVERRIDE KAPPA0_XY_OVERRIDE N_XY_OVERRIDE ...
           H_RD_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE SREF_GAMMA DH_D_CAP DH_D_TAU ...
           CBF_NO_REVERSE DIR_INSET_RELAX DRIFT_RECENTER DRIFT_GATED;
    cfg = WS.cfgs{ci};
    fprintf('\n=== seed %d/%d  cfg=%s ===\n', s, WS.nseeds, cfg.name);
    IC_OVERRIDE = WS.ic; NOISE_OVERRIDE = 1; RNG_SEED_OVERRIDE = s;
    SEN_PS0_OVERRIDE       = getfield_or(cfg,'ps0',[]);
    SEN_PSINF_OVERRIDE     = getfield_or(cfg,'psinf',[]);
    SEN_GAMMAS_OVERRIDE    = getfield_or(cfg,'gammas',[]);
    SEN_IZETASMAX_OVERRIDE = getfield_or(cfg,'izmax',[]);
    SEN_RP_OVERRIDE        = getfield_or(cfg,'rp',[]);
    SEN_RI_OVERRIDE        = getfield_or(cfg,'ri',[]);
    SEN_RD_OVERRIDE        = getfield_or(cfg,'rd',[]);
    SEN_CONTAIN_MODE       = getfield_or(cfg,'contain',0);
    THETA_CAP_OVERRIDE     = getfield_or(cfg,'thetacap',[]);
    FOV_INSET_OVERRIDE     = getfield_or(cfg,'fovinset',[]);
    GAMMA_XY_OVERRIDE      = getfield_or(cfg,'gamxy',[]);
    E_XY_OVERRIDE          = getfield_or(cfg,'exy',[]);
    P_XY_OVERRIDE          = getfield_or(cfg,'pxy',[]);
    KAPPA0_XY_OVERRIDE     = getfield_or(cfg,'k0xy',[]);
    N_XY_OVERRIDE          = getfield_or(cfg,'nxy',[]);
    H_RD_OVERRIDE          = getfield_or(cfg,'hrd',[]);
    KR_OVERRIDE            = getfield_or(cfg,'kr',[]);
    KOMEGA_OVERRIDE        = getfield_or(cfg,'kom',[]);
    SREF_GAMMA             = getfield_or(cfg,'sref',[]);
    DH_D_CAP               = getfield_or(cfg,'dhcap',[]);
    DH_D_TAU               = getfield_or(cfg,'dhtau',[]);
    CBF_NO_REVERSE         = getfield_or(cfg,'norev',[]);
    DIR_INSET_RELAX        = getfield_or(cfg,'dirinset',[]);
    DRIFT_RECENTER         = getfield_or(cfg,'drift',[]);
    DRIFT_GATED            = getfield_or(cfg,'driftg',[]);
    WS.s = s; WS.ci = ci;

    try
      visualControl_IBVS_adaptive;
      ni = idx;
      met.xy = norm(I_p_c(1:2) - x_t(1:2,idx));
      met.vel= norm(I_v_c - dx_t(1:3,idx));
      met.prec = double(met.xy <= 0.08); met.soft = double(met.vel <= 0.2);
      met.landed = double(landed); met.fov = double(fov_fail); met.t = tRange(idx);
      Ssd = abs([squeeze(S_s(1,1,1:ni))'; squeeze(S_s(2,2,1:ni))']);
      met.ssat = mean(Ssd >= (1-0.05-1e-9), 2)';
      met.brc  = mean(sen_breach(:,1:ni), 2)';
      met.maxsen = max(abs(V_s_e_n(:,1:ni)), [], 2)';
      % BREACH RATIO r = |s_e_n|/p_s over time; r>=1 = breach of the funnel.
      ratio = abs(V_s_e_n(:,1:ni)) ./ max(p_s(:,1:ni), 1e-6);
      met.maxr = max(ratio, [], 2)';                  % per-axis worst ratio
      met.nobreach = double(all(met.maxr < 1.0));      % 1 = s_e_n stayed in p_s
      bt = find(any(ratio >= 1.0, 1), 1, 'first');
      met.brt = NaN; if ~isempty(bt); met.brt = tRange(bt); end
      % RECOVERY: did s_e_n return inside p_s after a breach? (final r < 1)
      met.finalr = max(ratio(:,end));
      met.recovered = double(met.nobreach==0 && all(ratio(:,end) < 1.0));
      met.err = '';
    catch ME
      met = struct('xy',NaN,'vel',NaN,'prec',0,'soft',0,'landed',0,'fov',0, ...
                   't',NaN,'ssat',[NaN NaN],'brc',[NaN NaN],'maxsen',[NaN NaN], ...
                   'maxr',[NaN NaN],'nobreach',0,'brt',NaN, ...
                   'finalr',NaN,'recovered',0, ...
                   'err',sprintf('%s (%s)',ME.message,ME.identifier));
      fprintf('  ERR: %s\n', met.err);
    end
    global WS;
    met.seed = WS.s; met.cfg = WS.cfgs{WS.ci}.name;
    % flyaway = didn't land soft+precise AND drifted/sped away (not a clean marginal)
    met.flyaway = double(~(met.landed==1 && met.prec==1 && met.soft==1) && ...
                         (met.xy > 0.3 || met.vel > 0.5 || met.fov==1));
    WS.R{end+1} = met;
    fprintf('  -> NOBREACH=%d maxr=[%.2f %.2f] brt=%.2f | land=%d fov=%d xy=%.3f P=%d S=%d fly=%d\n', ...
            met.nobreach, met.maxr(1), met.maxr(2), met.brt, ...
            met.landed, met.fov, met.xy, met.prec, met.soft, met.flyaway);
    s = WS.s; ci = WS.ci;
  end
end

% ===================== SUMMARY =====================
global WS;
R = WS.R;
ncfg = numel(WS.cfgs);
cfgnames = cellfun(@(c) c.name, WS.cfgs, 'uni', 0);
fprintf('\n===== IC5 BREACH VALIDATION (n=%d seeds, %d cfgs) — aim: keep s_e_n < p_s =====\n', WS.nseeds, ncfg);
% per-seed: B = no breach (s_e_n stayed in p_s); ! = breached.  + SP marker.
fprintf('seed | %s\n', strjoin(cellfun(@(n) sprintf('%-11s',n), cfgnames,'uni',0),' '));
for s = 1:WS.nseeds
  codes = cell(1,ncfg);
  for ci = 1:ncfg
    x = R{(s-1)*ncfg + ci};
    sp = (x.landed==1 && x.prec==1 && x.soft==1);
    if x.nobreach==1;        sym='B';   % stayed inside p_s
    elseif x.recovered==1;   sym='R';   % breached then RETURNED inside p_s
    else;                    sym='!';   % breached, never returned
    end
    codes{ci} = sprintf('%-12s', sprintf('%s mx%.1f fn%.2f %s', sym, max(x.maxr), x.finalr, char('.'*sp+' '*(~sp))));
  end
  fprintf('%3d  | %s\n', s, strjoin(codes,' '));
end
fprintf('---\n');
for ci = 1:ncfg
  nm = cfgnames{ci};
  rr = R(ci:ncfg:end);
  nb = sum(cellfun(@(x) x.nobreach==1, rr));
  rec= sum(cellfun(@(x) x.recovered==1, rr));
  brk= sum(cellfun(@(x) x.nobreach==0 && x.recovered==0, rr));   % breached, never returned
  sp = sum(cellfun(@(x) x.landed==1 && x.prec==1 && x.soft==1, rr));
  fprintf('  %-14s: NO-BREACH=%d  RECOVERED=%d  UNRECOVERED=%d  (SP=%d/%d)\n', ...
          nm, nb, rec, brk, sp, numel(rr));
end
fprintf('==========================================================\n');
save('../Datasets/sen_ic5_seedtest_last.mat','R');

function v = getenv_num(name, d)
  e = getenv(name); if isempty(e); v = d; else; v = str2double(e); end
end
function v = getfield_or(s, f, d)
  if isfield(s,f) && ~isempty(s.(f)); v = s.(f); else; v = d; end
end
