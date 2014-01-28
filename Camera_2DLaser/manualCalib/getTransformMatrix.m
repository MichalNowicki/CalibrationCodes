%Function [Phi,Delta]=getTransformMatrix(Pts,Ns,Ds) 
%return transformation matrix H from laser coordinates to camera coordinates
% Pts -- the laser points hitting on the checkboard.
%        n cells, each of which is 2xm matrix,recorresponding to m points (x,0,z) in laser coordinate system 
% Ns  -- The normal of the planar suface of the checkerboard 
%        3 x n matrix --corresponding to the planes where Pts from
% Ds  -- The distance from camera center to the checkboard plane

function [Phi,Delta]=getTransformMatrix(Pts, Ns, Ds, weights)

if nargin<4
    weights=ones(1,size(Pts,2))';
end

preH=getInitialTransformMatrix(Pts,Ns,Ds);

preH(:,2)=preH(:,2)/norm(preH(:,2));
preH(:,1)=preH(:,1)/norm(preH(:,1));

estRot=[preH(:,1),-cross(preH(:,1),preH(:,2)),preH(:,2)];
[U,S,V] = svd(estRot);
estRot = U*V';

initV(1:3)=rodrigues(estRot);
initV(4:6)=preH(:,3);
initV=initV';

vars.Pts=Pts;
vars.Ns=Ns;
vars.Ds=Ds;
vars.weights=weights;

ptsNum=0;
for i=1:length(Pts)
ptsNum=ptsNum+size(Pts{i},2);
end
%options = optimset('LevenbergMarquardt','on','LargeScale','off');
options = optimset('Algorithm','Levenberg-Marquardt','LargeScale','off');
V=fminunc(@estimateErr,initV,options,vars);
 
Phi=rodrigues(V(1:3))';
Delta=-Phi*V(4:end);


fval=estProjErr(Phi,Delta,Pts, Ns, Ds);
disp(['Average distance error : ',num2str(fval), 'm.']);

return

%------------------------get initial H -------------------------------------%
function H=getInitialTransformMatrix(Pts, Ns, Ds)

num=length(Pts);
exPts=[];
exNs=[];
exDs=[];

for i=1:num
    n=size(Pts{i},2);
    exPts=[exPts,Pts{i}];
    exNs=[exNs,repmat(Ns{i},[1,n])];
    exDs=[exDs,repmat(Ds{i},[1,n])];
    disp([i size(exPts,2) size(exNs,2) size(exDs,2)]);
end

exPts=[exPts;ones(1,length(exPts))];
exPts=exPts';

exPts=repmat(exPts,[1,3]);
Nx=repmat(exNs(1,:)',[1,3]);
Ny=repmat(exNs(2,:)',[1,3]);
Nz=repmat(exNs(3,:)',[1,3]);
exNs=[Nx,Ny,Nz];

M=exPts.*exNs;
h=M\exDs';
H=reshape(h,3,3)';
return
%------------------------residual function-------------------------------------%

function [err]=estimateErr(x,vars)

Pts=vars.Pts; 
Ns=vars.Ns; 
Ds=vars.Ds;

num=length(Pts);
R=rodrigues(x(1:3)); %rotation matrix

err=0; 
for i=1:num

n=size(Pts{i},2);
pts=[Pts{i}(1,:);zeros(1,n);Pts{i}(2,:)];
T=x(4:end);
T=repmat(T,[1,n]);
newPts=R*pts+T; %apply rotation and translation 

err=err+sum((Ns{i}'*newPts-Ds{i}).^2)*vars.weights(i);

end

%-----------------------projection error with estimated extrinsic parameters ------------------%
function [err]=estProjErr(Phi,Delta,Pts, Ns, Ds, Indx)

num=size(Pts,2);
if nargin<6
    Indx=1:num;
end;

R=Phi';
err=0; 
ptsCount=0;

for i=Indx
n=size(Pts{i},2);
pts=[Pts{i}(1,:);zeros(1,n);Pts{i}(2,:)];
T=-R*Delta;
T=repmat(T,[1,n]);
newPts=R*pts+T; %apply rotation and translation 
err=err+sum(abs(Ns{i}'*newPts-Ds{i}));
ptsCount=ptsCount+n;

end

err=err/ptsCount;

return
