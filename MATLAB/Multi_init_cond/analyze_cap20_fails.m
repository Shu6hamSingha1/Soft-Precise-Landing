load('../Datasets/validate_cap20.mat');   % R: per-job structs (cap20 ci=2)
nm = {'IC1','IC2','IC3','IC4','IC5'};
fprintf('\n--- cap20 jobs that did NOT land or NOT achieve SP ---\n');
fprintf('IC   noise seed | land fov  xy      vel    | why\n');
for j = 1:numel(R)
  x = R{j};
  if x.ci ~= 2; continue; end
  if x.land==1 && x.sp==1; continue; end
  blk = find(cellfun(@(y) y.ic==x.ic && y.ci==x.ci, R));
  pos = find(blk==j); if pos==1; sd=0; else; sd=pos-1; end
  if x.land==0; why = 'NO LAND (fov/runaway)';
  elseif x.xy>0.08 && x.vel>0.2; why = sprintf('not P+S (xy=%.3f vel=%.3f)', x.xy, x.vel);
  elseif x.xy>0.08; why = sprintf('not PRECISE (xy=%.3f > 0.08)', x.xy);
  elseif x.vel>0.2; why = sprintf('not SOFT (vel=%.3f > 0.2)', x.vel);
  else; why = '?'; end
  fprintf('%-4s  %d    %d   |  %d   %d  %6.3f %6.3f | %s\n', ...
          nm{x.ic}, x.noise, sd, x.land, x.fov, x.xy, x.vel, why);
end
fprintf('-----------------------------------------------------\n');
