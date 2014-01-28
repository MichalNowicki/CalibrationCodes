% function mapping(idx,Phi,Delta,K,kc);
% project the laser reading onto the camera image with given camera pose and intrinsic paramaters
% idx - index of the image and laser rangding in the data sequence.

function mapping(idx,Phi,Delta,K,kc);

if nargin<5;
    kc=zeros(5,1);
end

imgFiles=filesInPath('./','jpg'); % load image filenames in the path
logFiles=filesInPath('./','log'); % load laser reading filenames in the path

img=imread(imgFiles{idx});
log=readLaserLog(logFiles{idx});
mapLaserPts(img,log,Phi,Delta,K,kc);

