% function [lpts, err]=getLinePts(Pts)
% given that the checkerboard is placed right in front of the planar
% laser rangefinder, extract the laser points hit on the checkerboard 
% input: Pts -- [1x 180] distance measurements in meter
% output: lpts -- [2 x n] the laser points hit on the checkerboard plane
%         err -- average fitting error of the laser points to the checkerboard plane (actually teh fitting line) 
function [lpts, err]=getLinePts(Pts)

theta=[0:179]/180*pi;
z=Pts.*sin(theta);
x=Pts.*cos(theta);

% plot(x,z,'b-');
% for i=1:2
% [xs(i),zs(i)]=ginput(1);
% hold on;plot(xs,zs,'ro-');hold off;
% end
% xs=sort(xs);
% valid=(x>=xs(1))&(x<=xs(2));
% 
% keyboard;

% the following lines choose the parts of the laser that are on the
% checkerboard.

f=[-1,1];
diffL=conv(Pts,f);
diffL=abs(diffL(2:181));

for i=95:-1:1
     if diffL(i)>0.25
        lowB=i+1;
        break;
    end
end;

for i=96:1:180
     if diffL(i)>0.25
        highB=i;
        break;
    end
end;

valid=lowB:highB;

%  alternatively, we can do this interactively.

figure(1);
P = [x(:) z(:)];
plot(x(:),z(:),'b.'); hold on;
plot(x(valid),z(valid),'r-'); hold off;
axis([-10 10 0 10]);

title(['select first (left click) and last (right click) points on' ...
       ' board, and click any key to continue.']);
BUTTON = 1;
while BUTTON < 4
  [X,Y,BUTTON] = GINPUT(1);
  if BUTTON < 4
    D = L2_distance([X Y]',P');
    [jnk idx] = min(D);
    switch BUTTON
     case {1}
      highB = idx(1);
     otherwise
      lowB = idx(1);
    end
    valid=lowB:highB;
    plot(x(:),z(:),'b.'); hold on;
    plot(x(valid),z(valid),'r-'); hold off;
    axis([-10 10 0 10]);
  end
end
    
valid=lowB:highB;
width=(highB-lowB+1);

x=x(valid);
z=z(valid);
pts=Pts(valid);

vtheta=theta(valid);

coef=polyfit(x,z,1);

x=coef(2)./(tan(vtheta)-coef(1));
z=tan(vtheta).*x;

if (nargout>1)
    err=sum(abs(pts-sqrt(x.^2+z.^2)))/length(x);
end 

z=Pts(valid).*sin(theta(valid));
%z=polyval(coef,x);

lpts=[x;z];

figure(2);
plot(Pts.*cos(theta),Pts.*sin(theta),x,z,'r.');






    