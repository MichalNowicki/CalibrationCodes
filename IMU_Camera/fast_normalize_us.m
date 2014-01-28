% fast simplified version of normalize_us
%
% unit sphere normalized pixel vector
%
% adapted from normalize (http://www.vision.caltech.edu/bouguetj/calib_doc/index.html)
%
% jlobo April 2004

function [Xn] = fast_normalize_us(x_kk,fc,cc,kc,alpha_c),

% fast simplified version of normalize_us
%
% unit sphere normalized pixel vector
%
% adapted from normalize (http://www.vision.caltech.edu/bouguetj/calib_doc/index.html)
%
% jlobo April 2004
%
%[Xn] = normalize_us(x_kk,fc,cc,kc,alpha_c)
%
%Computes the unit sphere normalized coordinates xn given the pixel coordinates x_kk
%and the intrinsic camera parameters fc, cc and kc.
%
%INPUT: x_kk: Feature locations on the images
%       fc: Camera focal length
%       cc: Principal point coordinates
%       kc: Distortion coefficients
%       alpha_c: Skew coefficient
%
%OUTPUT: Xn: Normalized feature locations on the unit sphere (a 3XN matrix)
%
%Important functions called within that program:
%
%comp_distortion_oulu: undistort pixel coordinates.

% if nargin < 5,
%    alpha_c = 0;
%    if nargin < 4;
%       kc = [0;0;0;0;0];
%       if nargin < 3;
%          cc = [0;0];
%          if nargin < 2,
%             fc = [1;1];
%          end;
%       end;
%    end;
% end;


%% First: Subtract principal point, and divide by the focal length:
%x_distort = [(x_kk(1,:) - cc(1))/fc(1);(x_kk(2,:) - cc(2))/fc(2)];

% % Second: undo skew
% x_distort(1,:) = x_distort(1,:) - alpha_c * x_distort(2,:);
% 
% if norm(kc) ~= 0,
% 	% Third: Compensate for lens distortion:
% 	x_n = comp_distortion_oulu(x_distort,kc);
% else
%    x_n = x_distort;
% end;

% xn are 2D image points on the plane at z=1
% project to unit sphere
Xn=[(x_kk(1,:) - cc(1))/fc(1);(x_kk(2,:) - cc(2))/fc(2); ones(size(x_kk(1,:)))];
Xn(3,:)=( Xn(1,:).^2 + Xn(2,:).^2 + Xn(3,:) ).^(-.5);
Xn(1,:)=Xn(1,:).*Xn(3,:);
Xn(2,:)=Xn(2,:).*Xn(3,:);