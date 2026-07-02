clc; clear; addpath('../Common');
global VDF_OVERRIDE STRESS_SCALE
p0=[0,0,-5;2,2,-5;2,-2,-5;2,2,-7;2,2,-3];
trajList=["Static","Linear","Sinusoidal","Lissajous","Circular"];
cfgs={'baked',struct();
      'Gxy.25',struct('Gamma',diag([0.25,0.25,0.75]));
      'Xih.7/1',struct('Xi_h',diag([0.7,0.7,1.0]));
      'both',struct('Gamma',diag([0.25,0.25,0.75]),'Xi_h',diag([0.7,0.7,1.0]))};
scells={'Sinusoidal',2;'Sinusoidal',4;'Sinusoidal',5;'Circular',3;'Circular',5};
fprintf('PX4-port A/B (full 25-cell realistic gate + 7x stress):\n');
fprintf('  %-8s | gate  | meanXY  worstXY | mean|v| | 7x SP\n','config');
for k=1:size(cfgs,1)
  VDF_OVERRIDE=cfgs{k,2}; STRESS_SCALE=[]; g=0; xy=[]; vv=[];
  for t=1:numel(trajList)
    for ic=1:5
      r=run_simulation([p0(ic,:)';1;0;0;0;zeros(6,1)],trajList(t),[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
      g=g+(r.precise&&r.soft); xy(end+1)=r.final_xy; vv(end+1)=r.final_rel_vel;
    end
  end
  sp7=0;
  for c=1:5
    VDF_OVERRIDE=cfgs{k,2}; STRESS_SCALE=7;
    r=run_simulation([p0(scells{c,2},:)';1;0;0;0;zeros(6,1)],scells{c,1},[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
    sp7=sp7+(r.precise&&r.soft);
  end
  fprintf('  %-8s | %2d/25 | %.4f  %.4f | %.4f | %d/5\n', cfgs{k,1}, g, mean(xy), max(xy), mean(vv), sp7);
end
clear global VDF_OVERRIDE STRESS_SCALE
