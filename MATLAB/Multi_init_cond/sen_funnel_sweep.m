% SEN_FUNNEL sweep — diagnose + tune the lateral demand-starvation that
% deferred IC5 [2,2,-3] (UBUNTU_HANDOFF.md §5 / FUNNEL_CBF_DESIGN.md §9).
%
% Runs a set of named funnel CONFIGS across the canonical 5 ICs and reports,
% per (config, IC): landing outcome + the funnel-saturation diagnostics that
% pin the starvation (S_s residency at the clip, izeta_s windup, breach
% fraction, max |s_e_n|).  Drives the canonical script via globals; keeps all
% loop state in the global WS so the script's top-of-file `clear` can't wipe
% it (same pattern as phase1_baseline_sweep.m).
%
% Edit WS.configs / WS.noise / WS.nreps below, then:  run sen_funnel_sweep

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|SEN_.*|WS).*$';
close all; clc;

global IC_OVERRIDE NOISE_OVERRIDE WS ...
       SEN_PS0_OVERRIDE SEN_PSINF_OVERRIDE SEN_GAMMAS_OVERRIDE ...
       SEN_IZETASMAX_OVERRIDE SEN_CONTAIN_MODE SEN_RP_OVERRIDE;

WS = struct();

% Canonical 5 ICs (UBUNTU_HANDOFF.md): IC5 [2,2,-3] is the deferred failure.
WS.ICs   = {[0;0;-5], [2;2;-5], [2;-2;-5], [2;2;-7], [2;2;-3]};
WS.ICnm  = {'IC1','IC2','IC3','IC4','IC5'};

% Funnel CONFIGS to compare. Each: name + override fields ([] = default).
%   ps0/psinf = 2x1, gammas = 2x1, izmax = scalar, contain = 0|1, rp = 2x1.
WS.configs = { ...
  struct('name','baseline',  'ps0',[],        'psinf',[],          'gammas',[],       'izmax',[], 'contain',0), ...
};

WS.noise = getenv_num('SEN_SWEEP_NOISE', 0);   % 0 noiseless / 1 noisy
WS.nreps = getenv_num('SEN_SWEEP_NREPS', 1);   % reps per (config,IC); >1 only useful w/ noise

WS.R = {};   % accumulated rows

for ci = 1:numel(WS.configs)
  for ic = 1:numel(WS.ICs)
    for r = 1:WS.nreps
      cfg = WS.configs{ci};
      fprintf('\n=== cfg=%-10s %s rep %d/%d noise=%d ===\n', ...
              cfg.name, WS.ICnm{ic}, r, WS.nreps, WS.noise);

      % --- set globals consumed by the canonical script ---
      IC_OVERRIDE    = WS.ICs{ic};
      NOISE_OVERRIDE = WS.noise;
      SEN_PS0_OVERRIDE       = getfield_or(cfg,'ps0',[]);
      SEN_PSINF_OVERRIDE     = getfield_or(cfg,'psinf',[]);
      SEN_GAMMAS_OVERRIDE    = getfield_or(cfg,'gammas',[]);
      SEN_IZETASMAX_OVERRIDE = getfield_or(cfg,'izmax',[]);
      SEN_RP_OVERRIDE        = getfield_or(cfg,'rp',[]);
      SEN_CONTAIN_MODE       = getfield_or(cfg,'contain',0);
      WS.ci = ci; WS.ic = ic; WS.r = r;

      try
        visualControl_IBVS_adaptive;   %#ok<*NASGU>
        % ---- workspace now holds the run; extract metrics ----
        ni      = idx;
        % NB: use `met` not `m` — the canonical script defines `m` = quadrotor
        % MASS (Constants.m), so `m.alt=...` would hit structAssToNonStruct.
        met.alt   = abs(I_p_c(3) - x_t(3,idx));
        met.xy    = norm(I_p_c(1:2) - x_t(1:2,idx));
        met.vel   = norm(I_v_c - dx_t(1:3,idx));
        met.prec  = double(met.xy <= 0.08);
        met.soft  = double(met.vel <= 0.2);
        met.landed= double(landed);
        met.fov   = double(fov_fail);
        met.fovt  = fov_fail_t;
        met.t     = tRange(idx);
        Ssd     = abs([squeeze(S_s(1,1,1:ni))'; squeeze(S_s(2,2,1:ni))']);
        met.ssat  = mean(Ssd >= (1 - 0.05 - 1e-9), 2)';        % clip residency per axis
        met.brc   = mean(sen_breach(:,1:ni), 2)';              % breach fraction per axis
        izm     = K_ctrl.izeta_s_max;
        met.izsat = mean(abs(izeta_s(:,1:ni)) >= 0.99*izm, 2)';% izeta windup residency
        met.maxsen= max(abs(V_s_e_n(:,1:ni)), [], 2)';         % peak normalized error
        bt = find(any(sen_breach(:,1:ni),1), 1, 'first');
        met.brt   = NaN; if ~isempty(bt); met.brt = tRange(bt); end
        met.err   = '';
      catch ME
        met = struct('alt',NaN,'xy',NaN,'vel',NaN,'prec',0,'soft',0,'landed',0, ...
                   'fov',0,'fovt',NaN,'t',NaN,'ssat',[NaN NaN],'brc',[NaN NaN], ...
                   'izsat',[NaN NaN],'maxsen',[NaN NaN],'brt',NaN, ...
                   'err',sprintf('%s (%s)',ME.message,ME.identifier));
        fprintf('  ERR: %s\n', met.err);
      end

      % canonical script `clear` just wiped our locals -> re-declare + restore
      global WS;
      cci = WS.ci; cic = WS.ic; cr = WS.r;
      met.cfg = WS.configs{cci}.name; met.icnm = WS.ICnm{cic}; met.rep = cr;
      WS.R{end+1} = met;
      fprintf(['  -> land=%d fov=%d t=%.2f xy=%.4f vel=%.4f P=%d S=%d | ' ...
               'ssat=[%.2f %.2f] izsat=[%.2f %.2f] brc=[%.2f %.2f] ' ...
               'maxsen=[%.2f %.2f] brt=%.2f\n'], ...
              met.landed, met.fov, met.t, met.xy, met.vel, met.prec, met.soft, ...
              met.ssat(1), met.ssat(2), met.izsat(1), met.izsat(2), ...
              met.brc(1), met.brc(2), met.maxsen(1), met.maxsen(2), met.brt);
      ci = cci; ic = cic; r = cr;
    end
  end
end

% ===================== SUMMARY =====================
global WS;
R = WS.R;
fprintf('\n================= SEN_FUNNEL SWEEP SUMMARY (noise=%d, nreps=%d) =================\n', WS.noise, WS.nreps);
fprintf('%-10s %-4s | land fov  t     xy      vel    P S | ssatX ssatY izsat brcX brcY maxsenX/Y  brt\n', 'cfg','IC');
for k = 1:numel(R)
  met = R{k};
  fprintf('%-10s %-4s |  %d   %d  %5.2f %6.3f %6.3f %d %d | %.2f  %.2f  %.2f  %.2f %.2f  %.2f/%.2f  %5.2f\n', ...
          met.cfg, met.icnm, met.landed, met.fov, met.t, met.xy, met.vel, met.prec, met.soft, ...
          met.ssat(1), met.ssat(2), max(met.izsat), met.brc(1), met.brc(2), ...
          met.maxsen(1), met.maxsen(2), met.brt);
end
fprintf('===============================================================================\n');
save('../Datasets/sen_funnel_sweep_last.mat','R');

% ---- helpers ----
function v = getfield_or(s, f, d)
  if isfield(s,f) && ~isempty(s.(f)); v = s.(f); else; v = d; end
end
function v = getenv_num(name, d)
  e = getenv(name); if isempty(e); v = d; else; v = str2double(e); end
end
