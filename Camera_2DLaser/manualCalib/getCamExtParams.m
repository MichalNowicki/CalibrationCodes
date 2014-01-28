% Estimate the extrinsic parameters of the camera with respect to the checkboard pattern
% function [R,T,imgPts,boardPts]=getCamExtParams(im,K,kc)
% input:  im--image that contains the checkerboard; 
%         K -- camera intrinsic matrix
%         kc--radius and tangent distortion parameters
% output: camera rotation R and translation T respective to the checkerboard plane
%         imgPts   -the grid points projected on the image
%         boardPts -the grid points on the checkerboard 

function [R,T,imgPts,boardPts]=getCamExtParams(img,K,kc)

 global sq_size;
 if nargin<3
    kc=zeros(5,1);
 end
 

 sq_size=100/1000;   %define checkboard square size -- 76mm  
 if ~exist('sq_size')|isempty(sq_size)
    sq_size=input('The length of the square:[default 76mm]:');
    if isempty(sq_size)
       sq_size=76/1000;   %76mm
    end
 end
 
 
 
 % extract the targets located on the corners of the checkerboard 
 targets=detecttargets(img); 
 
 if length(targets)~=4
     error('failed to estimate the targets');
 end
 
 % extract projected grid points along with their coords on the checkerboard
 [imgPts,boardPts]=getGridPoints(img,targets);  
 
 boardPts=boardPts*sq_size;
 
 % estimate camera extrinsic paramters -- R and T
 [omc,T,R,H] = compute_extrinsic(imgPts,boardPts,[K(1,1);K(2,2)],[K(1,3);K(2,3)],kc,K(1,2)/K(1,1));
 
 return
 
 %-------------test----------------------%
 %N=-R(:,3);
 %dist=-R(:,3)'*T; 
  % reproject the grid points onto the image with estimated extrinsic parameters
 [projPts] = project_points2(boardPts,omc,T,[K(1,1);K(2,2)],[K(1,3);K(2,3)],kc,K(1,2)/K(1,1));
 
 imshow(img);
 hold on;
 plot(projPts(1,:),projPts(2,:),'r.');
 hold off; 
  
  
 
     
