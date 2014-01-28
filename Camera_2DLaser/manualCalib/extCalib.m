% [Phi,Delta]=extCalib(K,kc)
% automatically estimate the relative position of camera with respect to laser ranger finder
% input: K  -- camera intrinic matrix; 
%        kc -- camera radius and tangent distortion parameters
% output: Phi -- 3x3 orthonormal matrix responding to camera orientation w.r.t laser range finder
%         Delta -- 3-vector, camera position w.r.t laser range finder  
% the function assumes that all image and laser data is in the current working directory.
% image data is JPEG file and laser data is postfixed as '.log'. 

function [Phi,Delta]=extCalib(K,kc)



global sq_size;
	
if nargin<2
    kc=zeros(5,1);
end;

imgFiles=filesInPath('./','png'); % load image filenames in the path
logFiles=filesInPath('./','log'); % load laser reading filenames in the path

num=length(imgFiles);


Pts=[];Ns=[];Ds=[];

for i=1:num
    disp(['Processing pose ',num2str(i),' ...']);
    img=imread(imgFiles{i});
    
    
    
    log=readLaserLog(logFiles{i});  
    [lpts,errs(i)]=getLinePts(log); % extract the laser points hitting on the checkboard
    Pts{i}=lpts; 
    
    % estimate extrinsic camera paramters w.r.t world coordinate system
    [extR,extT]=getCamExtParams(img,K,kc);
    
    Ns{i}=-extR(:,3); % checkerboard plane norm
    Ds{i}=-extR(:,3)'*extT; % distance of the plane to camera

end
%compute weights
weights=(1-errs.^10/sum(errs.^10))*num/(num-1);

disp('Calculating camera extrinsic parameters ');
[Phi,Delta]=getTransformMatrix(Pts, Ns, Ds,weights);

for i=1:num
    img=imread(imgFiles{i});
    log=readLaserLog(logFiles{i});
    
    mapLaserPts(img,log,Phi,Delta,K,kc);
    pause;
end;

% n=length(Pts);
% 
% thred=1;
% 
% while (thred>0.0001)
%     valid=[];
%     for i=1:n
%         err=getProjectionErr(Phi,Delta,Pts, Ns, Ds, i)
%         if (err<0.05)
%             valid=[valid,i];
%         end
%     end
%     if (length(valid)<=5) 
%         return;
%     end;
%     weights=(1-errs(valid)/sum(errs(valid)))*length(valid)/(length(valid)-1);
%     [phi,delta]=getTransformMatrix(Pts(valid), Ns(valid), Ds(valid),weights);
%     thred=norm([rodrigues(phi)-rodrigues(Phi);delta-Delta]);
%     Phi=phi;
%     Delta=delta;
% end;

clear global sq_size;
return






