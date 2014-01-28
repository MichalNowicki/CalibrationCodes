% load_imu_w
%
%  read inertial verticals from "standard" imu data text file:
% stored as text file with  ax ay az .... on each line, minimum 10 lines
% load_imu will read 1st 3 values on from the 1st 10 lines as the x, y,z acceleration in m.s^2
%
% calculate  weights (fixed maxang=2.5deg)
%
% [imu,imu_w]=load_imu(file_name,n)
%
% jlobo Jun 2008   updated to use weights and imu text file input
% jlobo April 2004

function [imu,imu_w]=load_imu(file_name,n)

maxang=2.5; %maxang err of 2deg, reject if above

imu=[];
imu_ae=[];
for i=1:n
   % load IMU data from text file
   %IMUTXT=load(sprintf('imu_%03d.txt',i));
   IMUTXT=load(sprintf('imu%03d.log',i));
   %[lines cols]=size(IMUTXT);
   lines=10;   % get 1st 10 values
   ax=-IMUTXT(1:lines,1);
   ay=-IMUTXT(1:lines,2);
   az=-IMUTXT(1:lines,3);
   % convert from m/s2 to gs
   % physicists have arrived at a standard value for g of 9.80665 m/s².
   % http://www.daviddarling.info/encyclopedia/A/accgrav.html
   %ax=ax/9.80665;
   %ay=ay/9.80665;
   %az=az/9.80665;
   % get mean value 
   m_ax=mean(ax);
   m_ay=mean(ay);
   m_az=mean(az);
   ang=[];
   for j=1:lines
      ang=[ang; vect_ang([ax(j) ay(j) az(j)],[m_ax m_ay m_az])];
   end
   %std_ang=std(ang);  this is wrong sinve 'ang' is already a deviation, so use rms
   %To obtain the root-mean-square (RMS) value, use norm(A)/sqrt(n)
   % norm(vector)sum(abs(A).^2)^(1/2), i.e. sqrt(mean(squares))
   std_ang=norm(ang)/sqrt(lines);
   % concatenate to final vectors
   imu=[imu; m_ax m_ay m_az];
   imu_ae=[imu_ae; std_ang];
end


% weight=1-ang_err_std/ang_max if >=0
maxang=pi*maxang/180;
imu_w=1-imu_ae./maxang;
imu_w(imu_w<0)=0;

%% un- comment to have optional plot
deg_ae=imu_ae.*180/pi;
plot(deg_ae,'o-');
hold on;
plot(imu_w,'*-r');
xlabel('Frame number');
ylabel('deg');
legend('o- Error (deg), *- Weight');
title('IMU ERROR');
hold off;