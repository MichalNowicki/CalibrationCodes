% calc_cam_vp()
%
% calculate image vanishing points
%
% jlobo mar 2004

% get vanishing point for each frame
cam=[];
for n=1:frames
    % vp direct from camera extrinsic
    eval(['vp=Rc_' num2str(n) '(:,2);']); % Y chessboard axis is vertical
    cam=[cam; vp'];
end



