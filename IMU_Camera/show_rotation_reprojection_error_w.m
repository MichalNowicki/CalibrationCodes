% show_rotation_reprojection_error
%
% show missmatch between observated and rotated vertical
%
% assumes the following variables exist:
%       Calib_Results have been loaded
%       cam -vertical vanishing points (normalized to unit sphere)
%       imu -inertial unit verticals
%       qw - imu to cam rotation quaternion
%       frames - frame from set
%
% uses:
%       vect_ang
%
% jlobo April 2004
 
%show_rotation_reprojection_error;
figure('Name','Rotation W Reprojection Error');
angerr=[];
for i=1:frames
angerr=[angerr vect_ang(qw*imu(i,:)',cam(i,:))];
 if angerr(i)>pi/2
	 angerr(i)=angerr(i)-pi;
 end
end
%To obtain the root-mean-square (RMS) value, use norm(A)/sqrt(n)
%matlab function ref: http://www-ccs.ucsd.edu/matlab/techdoc/ref/norm.html
% norm(vector)sum(abs(A).^2)^(1/2), i.e. str(mean(squares))
rms_angerr=norm(angerr)/sqrt(frames);
plot(abs(angerr*180/pi),'r*');
title(sprintf('Root Mean Square Angle Error %.3f (deg)',180*rms_angerr/pi));
set(gca,'xtick',image_numbers);
set(gca,'YGrid','on');
xlabel('Frames');
ylabel('Error (deg)');
