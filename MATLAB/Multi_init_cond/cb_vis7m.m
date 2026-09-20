clc; clear; addpath('../Common');
f=135; res=[320;240]; dt=1/30; ns=2000; rng(1);
base=[15/sqrt(2),-15/sqrt(2),-15/sqrt(2),15/sqrt(2),22;15/sqrt(2),-15/sqrt(2),15/sqrt(2),-15/sqrt(2),0;0 0 0 0 0]*2/250;
base=[base(1:2,:)-[0;0]*0; zeros(1,5)]; % origin = cross intersection
vc=[0.5;0.3;-0.5]; wz=0.1;   % camera velocity (level camera, yaw rate)
for Z=[7 5]
 for sc=[1 6 12 24 32]
  T=base*sc; off=[2;2];  % worst lateral offset seen at start (IC2)
  proj=@(p,c,psi) f*[cos(psi) sin(psi);-sin(psi) cos(psi)]*(p(1:2,:)-c(1:2))./(Z+0*p(1,:));
  P0=proj(T,[off;0],0); ext=max(P0,[],2)-min(P0,[],2); inn=all(all(abs(P0)<res/2));
  ctr=proj([0;0;0],[off;0],0);
  % true optic-flow quantities: camera moves by vc*dt, yaws wz*dt
  Pn=@(k) f*(rotz_(-wz*dt*k)*(T(1:2,:)-(off+vc(1:2)*dt*k)))./(Z+vc(3)*dt*k);
  Pa=Pn(0); Pb=Pn(1);
  L=@(P) buildL(P,f);
  Lm=L(Pa); dP=reshape(Pb-Pa,[],1)/dt; sol0=lsqminnorm(Lm(:,[1 2 3 6]),dP);
  sv=svd(Lm(:,[1 2 3 6]));
  e=zeros(4,ns); al=zeros(1,ns); sg=0.027+0.175/(Z+0.5);
  for i=1:ns
    A=Pa+sg*randn(2,5); B=Pb+sg*randn(2,5); LA=L(A); s4=lsqminnorm(LA(:,[1 2 3 6]),reshape(B-A,[],1)/dt);
    e(:,i)=s4-sol0; sa=image_feature(A/f); al(i)=sa(4);
  end
  a0=image_feature(Pa/f); 
  fprintf('Z=%g scale=%2dx: extent=%5.1f x %5.1f px | all5 in frame=%d ctr=(%.0f,%.0f)px | sv(min..max)=%.2g..%.2g | noise: rms err h_xy=%.3f h_z=%.3f w_z=%.3f (true |h_xy|=%.3f h_z=%.3f w_z=%.2f) | alpha std=%.2f deg\n',...
   Z,2*sc,ext(1),ext(2),inn,ctr(1),ctr(2),min(sv),max(sv),rms(e(1:2,:),'all'),rms(e(3,:)),rms(e(4,:)),norm(sol0(1:2)),sol0(3),sol0(4),std(al)*180/pi);
 end
end
function R=rotz_(a), R=[cos(a) -sin(a);sin(a) cos(a)]; end
function L=buildL(P,f)
 L=zeros(2*size(P,2),6);
 for j=1:size(P,2), x=P(1,j);y=P(2,j);
  L(2*j-1:2*j,:)=[f 0 -x -x*y/f (f^2+x^2)/f -y; 0 f -y -(f^2+y^2)/f x*y/f x]; end
end
