% estimate the relative position of camera with respect to laser ranger finder
% K -- camera intrinic matrix; kc--camera radius and tangent distortion parameters

function [Phi,Delta]=CalibExtParam(K,kc,imgPts,boardPts)

logFiles=filesInPath('./','log'); % load laser reading filenames in the path

num=length(logFiles);

Pts=[];

disp(['loading laser readings ']);
for i=1:num
    log=readlaserlog(logFiles{i});  
    [lpts,errs(i)]=getLinePts(log); % extract the leaser points hitting on the checkboard
    Pts{i}=lpts;
end

indx=1:num;
[Phi, Delta]=calExtr(K,kc,imgPts(indx),boardPts(indx),Pts(indx),errs(indx));

return

%-------------------------------------------------------------------------
function [Phi, Delta]=calExtr(K,kc,imgPts,boardPts,Pts,errs)

num=length(imgPts);
Ns=[];
Ts=[];

for i=1:num
    
    [omc,T,R,H] = compute_extrinsic(imgPts{i},boardPts{i},[K(1,1);K(2,2)],[K(1,3);K(2,3)],kc,0);
    Ns{i}=-R(:,3);
    Ds{i}=-R(:,3)'*T;
    
    Rs{i}=R;
    Ts{i}=T;

end

%--------------estimate camera extrinsic parameters relative to laser range finder  -------%
weights=(1-errs.^10/sum(errs.^10))*num/(num-1);
[Phi,Delta]=getTransformMatrix(Pts, Ns, Ds,weights);

return
