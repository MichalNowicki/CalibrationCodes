% load_imu
%
%  read inertial verticacls from "standard" imu data text file:
% stored as text file with  ax ay az .... on each line, minimum 10 lines
% load_imu will read 1st 3 values on from the 1st 10 lines as the x, y,z acceleration in m.s^2
%
% [vert]=load_imu(file_name,n)
%
% jlobo April 2004

function [vert]=load_imu(file_name,n)

vert=[];
for i=1:n
   % load(sprintf('%simu_%03d',file_name,i));
   load(sprintf('imu_%03d',i));
   
   [lines cols]=size(imu);
   % get sensor raw data
   ax=mean(imu(1,:));
   ay=mean(imu(2,:));
   az=mean(imu(3,:));
   % convert from m/s2 to gs
   % physicists have arrived at a standard value for g of 9.80665 m/s².
   % http://www.daviddarling.info/encyclopedia/A/accgrav.html
   %ax=ax/9.80665;
   %ay=ay/9.80665;
   %az=az/9.80665;
   vert=[vert; ax ay az];
end
