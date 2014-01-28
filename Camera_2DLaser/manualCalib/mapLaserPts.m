% function mapLaserPts(img,log,phi,delta,K,kc)
% project the laser reading onto the camera image with given camera pose and intrinsic paramaters
% img - camera image
% log - laser reading
% phi, delta - camera orientation and position
% K, kc - camera internal matrix and distortion paramaters

function mapLaserPts(img,log,phi,delta,K,kc)

if nargin<5
    kc=zeros(5,1);
end;

[height,width]=size(img);

%theta=0:179;
%theta=theta/180*pi;
%theta=[0:0.25:180]/180*pi;
theta=[-45:0.25:225]/180*pi;

log = log./1000;

x=log.*cos(theta);
z=log.*sin(theta);

phi=phi';
delta=-phi*delta;

Pts=[x;zeros(1,length(theta));z];
t=repmat(delta,[1,length(log)]);
p=phi*Pts+t;

%p=p(:,300:400);

% p=p./repmat(p(3,:),[3,1]);
% 
% r=sum(p(1:2,:).^2);
% p(1,:)=p(1,:).*( 1+kc(1)*r+kc(2)*r.^2+kc(5)*r.^3 )+ 2*kc(3)*p(1,:).*p(2,:) + kc(4)*(r + 2*p(1,:).^2);;
% p(2,:)=p(2,:).*( 1+kc(1)*r+kc(2)*r.^2+kc(5)*r.^3 )+ kc(3)*(r + 2*p(2,:).^2) + 2*kc(4)*p(1,:).*p(2,:);;
% pts=K*p;


[pts] = project_points2(p,zeros(3,1),zeros(3,1),[K(1,1);K(2,2)],[K(1,3);K(2,3)],kc,K(1,2)/K(1,1));

valid=(pts(1,:)<width)&(pts(1,:)>0)&(pts(2,:)<height)&(pts(2,:)>0);

figure,imshow(img);
hold on, plot(pts(1,valid),pts(2,valid),'r.','MarkerSize',20);
hold off;



