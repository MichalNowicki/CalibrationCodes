% Estimate camera position w.r.t laser range finder and do
% global optimization on intrinsic and extrinsic camera parameters 

% K -- camera intrinic matrix; kc--camera radius and tangent distortion parameters

function [Phi,Delta, newK, newkc]=glbOptimize(K,kc)

if nargin<2
    kc=zeros(5,1);
end;

imgFiles=filesInPath('./','jpg'); % load image filenames in the path
logFiles=filesInPath('./','log'); % load laser reading filenames in the path

num=length(imgFiles);

Pts=[];Ns=[];Ds=[];

for i=1:num
    disp(['Processing image ',num2str(i),' ...']);
    img=imread(imgFiles{i});
    
    [extR,extT,gridPts,ptsCoord]=getCamExtParams(img,K,kc);
    Ns{i}=-extR(:,3);
    Ds{i}=-extR(:,3)'*extT;
    
    Rs{i}=extR;
    Ts{i}=extT;
    
    log=readlaserlog(logFiles{i});  
    [lpts,errs(i)]=getLinePts(log); % extract the leaser points hitting on the checkboard
    
    Pts{i}=lpts;
    imgPts{i}=gridPts;
    boardPts{i}=ptsCoord;
end

%--------------estimate camera extrinsic parameters relative to laser range finder  -------%
weights=(1-errs.^20/sum(errs.^20))*num/(num-1);
[Phi,Delta]=getTransformMatrix(Pts, Ns, Ds,weights);

%---------------do global optimization ---------------------------------%
inx=getXfromParameters(Phi,Delta,K,kc,Rs,Ts);
vars.imgPts=imgPts;
vars.boardPts=boardPts;
vars.Pts=Pts; 

options = optimset('LevenbergMarquardt','on','LargeScale','off','Display','iter');
x=fminunc(@estimateErr,inx,options,vars);

[invPhi,invDelta,newK,newkc,newRs,newTs]=getParamfromX(x);

Phi=invPhi';
Delta=-invPhi'*invDelta;


return


%-----------------------------------end of glbOptimize ----------------------------%

function [err]=estimateErr(x,vars)

imgPts=vars.imgPts;
boardPts=vars.boardPts;
Pts=vars.Pts; 

[invPhi,invDelta,K,kc,Rs,Ts]=getParamfromX(x);

num=length(Pts);
err=0; 

for i=1:num

n=length(Pts{i});
pts=[Pts{i}(1,:);zeros(1,n);Pts{i}(2,:)];

extR=Rs{i};
extT=Ts{i};

N=-extR(:,3);
D=-extR(:,3)'*extT;

projPts=[extR,extT]*[boardPts{i};ones(1,length(boardPts{i}))];
projPts=projPts./repmat(projPts(3,:),[3,1]);

%call r^2 = a^2 + b^2.
%The distorted point coordinates are: xd = [xx;yy] where:
%
%xx = a * (1 + kc(1)*r^2 + kc(2)*r^4 + kc(5)*r^6)      +      2*kc(3)*a*b + kc(4)*(r^2 + 2*a^2);
%yy = b * (1 + kc(1)*r^2 + kc(2)*r^4 + kc(5)*r^6)      +      kc(3)*(r^2 + 2*b^2) + 2*kc(4)*a*b;
%
%The left terms correspond to radial distortion (6th degree), the right terms correspond to tangential distortion
%
%Finally, convertion into pixel coordinates: The final pixel coordinates vector xp=[xxp;yyp] where:
%
%xxp = f(1)*(xx + alpha*yy) + c(1)
%yyp = f(2)*yy + c(2)


rdists=sum(projPts(1:2,:).^2);
projPts(1,:)=projPts(1,:).*(1+kc(1)*rdists+kc(2)*rdists.^2);
projPts(2,:)=projPts(2,:).*(1+kc(1)*rdists+kc(2)*rdists.^2);
projPts=K*projPts;

projErr=sum(sum((projPts(1:2,:)-imgPts{i}).^2))/length(imgPts{i});

delta=repmat(invDelta,[1,n]);
newPts=invPhi*pts+delta; %apply rotation and translation 

err=err+10*sum((N'*newPts-D).^2)+projErr;

end
return


%-----------------------------------------------------%
function x=getXfromParameters(Phi,Delta,K,kc,Rs,Ts)

x=[];
x(1:3)=rodrigues(Phi');
x(4:6)=-Phi'*Delta;

x(7)=K(1,1);x(8)=K(2,2);x(9)=K(1,3);x(10)=K(2,3);
x(7:10)=x(7:10)+0.25*randn(1,4);

x(11:14)=kc(1:4)+0.001*randn(1,4);
for i=1:length(Rs)
    x((15+6*(i-1)):(14+6*i))=[rodrigues(Rs{i});Ts{i}];
end

x=x';

return

%-------------------------------------------------------%
function [invPhi,invDelta,K,kc,Rs,Ts]=getParamfromX(x)

invPhi=rodrigues(x(1:3)); %rotation matrix
invDelta=x(4:6);

K=[x(7),0, x(9); 0, x(8), x(10); 0, 0, 1]; 

kc=zeros(5,1);

kc(1:4)=x(11:14);
for i=1:(length(x(15:end))/6)
    Rs{i}=rodrigues(x((15+6*(i-1)):(15+6*i-4)));
    Ts{i}=x((15+6*i-3):(15+6*i-1));
end

return

