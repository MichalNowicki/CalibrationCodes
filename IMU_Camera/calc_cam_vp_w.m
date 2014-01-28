% calc_cam_vp_w()
%
% calculate image vanishing points and weights (fixed maxang=2.5deg)
%
% jlobo mar 2004
% jlobo Set 2006: update with weights wi based on omc_error_i

maxang=2.5; %maxang err of 2.5 deg, reject if above

% get vanishing point for each frame
cam=[];
cam_ae=[];
for n=1:frames
    % vp direct from camera extrinsic
    eval(['vp=Rc_' num2str(n) '(:,2);']); % Y chessboard axis is vertical
    cam=[cam; vp'];    
    eval(['ae=norm(omc_error_' num2str(n) ');']); % magnitude of rotation error, aprox 3x stddev
    cam_ae=[cam_ae; ae];
end

% weight=1-ang_err_std/ang_max if >=0
maxang=pi*maxang/180;
cam_w=1-cam_ae./maxang;
cam_w(cam_w<0)=0;

%% un-comment to have optional plot
deg_ae=cam_ae.*180/pi;
plot(deg_ae,'o-');
hold on;
plot(cam_w,'*-r');
xlabel('Frame number');
ylabel('o- Error (deg), *- Weight');
hold off;
