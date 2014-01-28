function [err]=getProjectionErr(Phi,Delta,Pts, Ns, Ds, Indx)

num=length(Pts);
if nargin<6
    Indx=1:num;
end;

R=Phi';
err=0; 
ptsCount=0;

for i=Indx
n=length(Pts{i});
pts=[Pts{i}(1,:);zeros(1,n);Pts{i}(2,:)];
T=-R*Delta;
T=repmat(T,[1,n]);
newPts=R*pts+T; %apply rotation and translation 
err=err+sum(abs(Ns{i}'*newPts-Ds{i}));
ptsCount=ptsCount+n;

end

err=err/ptsCount;