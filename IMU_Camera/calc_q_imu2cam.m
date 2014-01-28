% calc_q_imu2cam()
%
% calculate rotation given IMU and CAM observations of vertical direction
%
% based on Horn's method
%
% requires robot toolbox by Peter Corke
%
% jlobo set 2003

function q = calc_q_imu2cam(imu,cam);

[lines cols]=size(imu);

% we now want to estimate the quaternion q that rotates imu to cam

% compute the sums of all 9 possible products between pairs of coordinates
Sxx=0;
Sxy=0;
Sxz=0;
Syx=0;
Syy=0;
Syz=0;
Szx=0;
Szy=0;
Szz=0;
for i = 1:lines
    Sxx=Sxx+imu(i,1)*cam(i,1);
    Sxy=Sxy+imu(i,1)*cam(i,2);
    Sxz=Sxz+imu(i,1)*cam(i,3);
    Syx=Syx+imu(i,2)*cam(i,1);
    Syy=Syy+imu(i,2)*cam(i,2);
    Syz=Syz+imu(i,2)*cam(i,3);
    Szx=Szx+imu(i,3)*cam(i,1);
    Szy=Szy+imu(i,3)*cam(i,2);
    Szz=Szz+imu(i,3)*cam(i,3);
end

A=[ (Sxx+Syy+Szz)       Syz-Szy         Szx-Sxz         Sxy-Syx     
    Syz-Szy             (Sxx-Syy-Szz)   Sxy+Syx         Szx+Sxz
    Szx-Sxz             Sxy+Syx         (-Sxx+Syy-Szz)  Syz+Szy
    Sxy-Syx             Szx+Sxz         Syz+Szy         (-Sxx-Syy+Szz) ];

% compute eigen values and vectors
[V,D] = eig(A);

% sort eigenvalues
lD=diag(D);
[B,index] = sort(abs(lD));

% choose the eigenvector corresponding to
% the largets eigen value
r = V(:,index(4));
q=Quaternion(r');
