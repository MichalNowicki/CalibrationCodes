% function [R,T,imgPts,boardPts]=getExtParameters(im,K,kc)
% input: im-- image that contains the checkerboard; 
%        K -- 3x3 camera intrinsic matrix
%        kc-- 5-vector, radius and tangent distortion parameters
% output: camera rotation R and translation T respective to the checkerboard plane
%         imgPts   -2xn matrix, the grid points projected on the image
%         boardPts -3xn matrix, the grid points on the checkerboard 

function [R,T,imgPts,boardPts]=getExtParameters(im,K,kc)

     global sq_size;
     
     if nargin<3
         kc=zeros(5,1);
     end
     
     %imshow(im);
     winz=5;  %window size
     disp('Please select a rectangle area from the checkerboard pattern.');
     [height, width]=size(im);
     figure(1);
     [bw,cornerX,cornerY]=roipoly(im);
     cornerX=cornerX(1:4);
     cornerY=cornerY(1:4);
     
     [Xc,good,bad,type] = cornerfinder([cornerX';cornerY'],im,winz,winz); % the four corners
     
    
     x = Xc(1,:)';
     y = Xc(2,:)';
     hold on; plot([x;x(1)],[y;y(1)],'ro-');hold off;
     
     n_sq_x1 = count_squares(im,x(1),y(1),x(2),y(2),winz);
     n_sq_x2 = count_squares(im,x(3),y(3),x(4),y(4),winz);
     n_sq_y1 = count_squares(im,x(2),y(2),x(3),y(3),winz);
     n_sq_y2 = count_squares(im,x(4),y(4),x(1),y(1),winz);

     
     if (n_sq_x1~=n_sq_x2)|(n_sq_y1~=n_sq_y2)
        sq_num_x=input('The number of squares along X axis:');
        sq_num_y=input('The number of squares along Y axis:');
     else
        sq_num_x=n_sq_x1;
        sq_num_y=n_sq_y1;
     end
     
    
     disp(['the size of the section is ',num2str(sq_num_x), 'x', num2str(sq_num_y) ]);
     if ~exist('sq_size')|isempty(sq_size)
        sq_size=input('The length of the square:[default 76mm]:');
        if isempty(sq_size)
            sq_size=76/1000;   %76mm
        end
     end
     [imgPts,boardPts]=projectGridPts(Xc,sq_num_x,sq_num_y);
     boardPts=boardPts*sq_size;
     
     [imgPts,good,bad,type] = cornerfinder(imgPts,im,winz,winz); 
     
     [omc,T,R,H] = compute_extrinsic(imgPts,boardPts,[K(1,1);K(2,2)],[K(1,3);K(2,3)],kc,K(1,2)/K(1,1));
     
     %N=-R(:,3)
     %dist=-R(:,3)'*T
     imgPts=imgPts(:,good);
     boardPts=boardPts(:,good);
     
     [projPts] = project_points2(boardPts,omc,T,[K(1,1);K(2,2)],[K(1,3);K(2,3)],kc,K(1,2)/K(1,1));

%      figure(2);
%      imshow(im);
%      hold on;
%      plot(projPts(1,:),projPts(2,:),'r+');
%      hold off; 
     
     return
     
  
%------------re-projected grid points of the checherboard---------------------------%

function [gridPts,ptsCoord]=projectGridPts(cornerPts,sqNumX,sqNumY)

     [Xs,Ys]=meshgrid(1:(sqNumX+1),1:(sqNumY+1)) ;
     ptsCoord=[Xs(:)-1,Ys(:)-1,zeros((sqNumX+1)*(sqNumY+1),1)]';
     
     Xs=(Xs(:)-1)/sqNumX+1;
     Ys=(Ys(:)-1)/sqNumY+1;
     
     Zx=[cornerPts(1,1) cornerPts(1,2);
         cornerPts(1,4) cornerPts(1,3)];
     
     Zy=[cornerPts(2,1) cornerPts(2,2);
         cornerPts(2,4) cornerPts(2,3)];
     
     gridPts(1,:)=interp2(Zx,Xs,Ys)';
     gridPts(2,:)=interp2(Zy,Xs,Ys)';
     
return
     
