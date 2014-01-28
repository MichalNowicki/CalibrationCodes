% [gridPts,gridCoords]=getGridPoints(img, targets);
% Detect grid points from the image and return their image coordinates and 
% word coordinates

function [gridPts,gridCoords]=getGridPoints(img, targets);

regImg=roipoly(img,[targets.center_x],[targets.center_y]);

origImg=img;
img=rgb2gray(img);
regImg= uint8(double(regImg).*double(img));


[cim, r, c] = harris(regImg, 3, 10000, 3, 0);

cornerNum=length(r);
rawPts=[c';r';ones(1,cornerNum)];

cPts=[[targets.center_x];[targets.center_y]; ones(1,length(targets))];
cPts=cPts(:,[1:length(targets),1]);

thrDist=5;
for i=1:length(targets)
    line=cross(cPts(:,i),cPts(:,i+1)); line=line/norm(line(1:2));
    dists=abs(line'*rawPts);
    rawPts=rawPts(:,dists>thrDist);
end

[gridPts,good,bad,type] = cornerfinder(rawPts(1:2,:),regImg,3,3);


x1=cPts(1:3,1:end-1);
x2=[0 0 1 1; 0 1 1 0; 1 1 1 1];

H = homography2d(x1, x2);

gridPts=[gridPts;ones(1,length(gridPts))];
hgrids=H*gridPts;
hgrids=hgrids./repmat(hgrids(3,:),[3,1]);

[ccol,col] = clusterPts(hgrids(1,:)); colNum=length(col);
[crow,row] = clusterPts(hgrids(2,:)); rowNum=length(row);

gridCoords=zeros(size(hgrids));

for i=1:length(hgrids)
    hgrids(1,i)=col(ccol(i));
    gridCoords(1,i)=colNum-ccol(i);
    hgrids(2,i)=row(crow(i));
    gridCoords(2,i)=rowNum-crow(i);
end

[results,indx]=sortrows(hgrids(2:-1:1,:)');

gridPts=gridPts(1:2, indx(end:-1:1));
good=good(indx(end:-1:1));
gridPts=gridPts(:,find(good));


gridCoords=gridCoords(:, indx(end:-1:1));
gridCoords=gridCoords(:, find(good));

% figure
% imshow(origImg);
% hold on;
% plot(gridPts(1,:),gridPts(2,:),'r.');
% for i=1:length(gridPts)
%     text(gridPts(1,i),gridPts(2,i),num2str(i),'Color','r')
% end

return

%----------------------------------------------------------------%

function [clusterIndx,kmc] = clusterPts(x)

n =length(x);

K=n;
kmc=x;

oldK=K;
clusterIndx = zeros(1,n);
D = zeros(K,n);

while(1),
   
   if oldK~=K 
      clusterIndx = zeros(1,n);
      D = zeros(K,n);
   end

   for j=1:K,                        % for every cluster
      D(j,:) = abs(x-kmc(j));
   end

   % find mini kmcm
   [Dmin,index] = min(D);
   moved = sum(index~=clusterIndx);
   clusterIndx = index;

   for i=1:K
      ci=find(clusterIndx==i);
      kmc(i)=mean(x(ci));
   end
   
   kmc=sort(kmc);
   nkmc=kmc(1);
   for i=1:K
       indx=length(nkmc);
       if abs(nkmc(end)-kmc(i))<0.05
          nkmc(end)=(nkmc(end)+kmc(i))/2;
          clusterIndx(find(clusterIndx==i))=indx;
      else
          nkmc=[nkmc,kmc(i)];
      end
   end
   
   kmc=nkmc;
   
   oldK =K;
   K=length(kmc);

   if (moved==0), return, end

end % while(1)
