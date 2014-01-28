% show_us_vp_cam_imu
%
% show unit sphere image and vertical vanishing point,
% inertial vertical, and its rotation to camera frame
%
% assumes:
%       Calib_Results have been loaded
%       cam -vertical vanishing points (normalized to unit sphere)
%       ins -inertial unit verticals (imu ref)
%       q -imu to cam rotation quaternion
%       n -current frame from set
%
% uses:
%       [xn] = normalize(x_kk,fc,cc,kc,alpha_c) from Camera Calibration toolbox
%       fscatter3
%       clickmove
%       arrow3
%
% jlobo June 2008: use 'opengl software' if rendering is faulty
% jlobo April 2004

% sphere overlay levels
a=1.02;
b=1.01;


% set image name and figure title for current frame n
filename=sprintf('%s%03d.bmp',calib_name,n);
figure('Name',sprintf('Frame %d : %s',n,filename));  

% simple names, not indexed by n
eval(['x=x_' num2str(n) ';']);

% setup figure with nice rendered shiny sphere
subplot('Position',[0.31 0 .69 1]) 
[X,Y,Z]=sphere(40);
H=surf(X,Y,Z,Z);
caxis([-1 1]);
set(H,'FaceAlpha',0.6)
set(H,'FaceLighting','phong','FaceColor','interp','AmbientStrength',0.5)
light('Position',[1 -1 0.5],'Style','infinite');
axis equal;
view(150,75);
shading interp;
    
% load frame image and show it at top left corner
I = imread(filename);
subplot('Position',[0.01 .69 .3 .3]) 
imshow(I),

% plot chessboard grid on image plane
hold on;
plot(x(1,:),x(2,:),'b+','markersize',5); % all grid points
plot(x(1,1),x(2,1),'ro','markersize',5);   % corner
plot(x(1,43),x(2,43),'yo','markersize',5); % corner: grid origin, marked yellow
plot(x(1,6),x(2,6),'ro','markersize',5);   % corner
plot(x(1,48),x(2,48),'ro','markersize',5); % corner
hold off;

% project chessboard grid to unit sphere
X_us=normalize_us(x,fc,cc,kc,alpha_c);

%find vp from 4 outer grid points
point_a=X_us(:,1);              
point_b=X_us(:,43);
point_c=X_us(:,6);
point_d=X_us(:,48);             % grid origin, marked yellow
line_m1=cross(point_a,point_b);
line_m1=unit(line_m1);
line_m2=cross(point_c,point_d);
line_m2=unit(line_m2);
vp=cross(line_m1,line_m2);
% vp direct from camera extrinsic
eval(['vp=Rc_' num2str(n) '(:,2)']); % Y chessboard axis is vertical

%plot chessboard grid on unit sphere    
subplot('Position',[0.31 0 .69 1]) 
hold on;
plot3(a*X_us(1,:),a*X_us(2,:),a*X_us(3,:),'b.','markersize',4);

%plot vanishing point, and its construction on unit sphere    
plot3(vp(1),vp(2),vp(3),'r.','markersize',20);
plot3(a*point_a(1),a*point_a(2),a*point_a(3),'r.','markersize',10);
plot3(a*point_b(1),a*point_b(2),a*point_b(3),'y.','markersize',10);  % grid origin, marked yellow
plot3(a*point_c(1),a*point_c(2),a*point_c(3),'r.','markersize',10);
plot3(a*point_d(1),a*point_d(2),a*point_d(3),'r.','markersize',10);  
circ=great_circ(point_a',point_b');
plot3(a*circ(:,1),a*circ(:,2),a*circ(:,3),'b-','linewidth',1.5);
circ=great_circ(point_c',point_d');
plot3(a*circ(:,1),a*circ(:,2),a*circ(:,3),'b-','linewidth',1.5);
circ=great_circ(vp');
plot3(a*circ(:,1),a*circ(:,2),a*circ(:,3),'r-','linewidth',2);
arrow3([0 0 0],1.3*vp','r',.1,.2);

%plot imu vertical on unit sphere
plot3(1.1*imu(n,1),1.1*imu(n,2),1.1*imu(n,3),'k.','markersize',10);
arrow3([0 0 0],1.3*imu(n,:),'k',.1,.1)
circ=great_circ(imu(n,:));
plot3(a*circ(:,1),a*circ(:,2),a*circ(:,3),'k:','linewidth',1);

%plot imu vertical rotated to camera frame on unit sphere
cimu=imu(n,:);
cimu(:)=(q*imu(n,:)');
plot3(1.2*cimu(1),1.2*cimu(2),1.2*cimu(3),'m.','markersize',10);
arrow3([0 0 0],1.4*cimu,'m',.1,.1)
circ=great_circ(cimu);
plot3(a*circ(:,1),a*circ(:,2),a*circ(:,3),'m:','linewidth',1);
hold off;

% subsample image and project to unit sphere
[umax,vmax,color_dim]=size(I);
res=5; %step controls image res
IPOINTS=ones(4,(umax/res)*(vmax/res)); %pre-allocate for faster matlab
mi=[0 0 0 0]';
i=1;
for u=1:res:umax
    for v=1:res:vmax    
        mi(1:3)=fast_normalize_us([v u]',fc,cc,kc,alpha_c); % fast model normalize to unit sphere
        mi(4)=I(u,v,1);    
        IPOINTS(:,i)=mi;
        i=i+1;
    end
end
fscatter3(b*IPOINTS(1,:),b*IPOINTS(2,:),b*IPOINTS(3,:),IPOINTS(4,:),gray(256));

% adjust view and set mouse click move
%set(gca,'CameraUpVector',cam(n,:));
set(gca,'CameraViewAngle',7);
axis tight;
axis off;
clickmove;
hold off;

 



