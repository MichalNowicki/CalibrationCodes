% NORMALISE2DPTS - normalises 2D homogeneous points
%
% Function translates and normalises a set of 2D homogeneous points 
% so that their centroid is at the origin and their mean distance from 
% the origin is sqrt(2).  This process typically improves the
% conditioning of any equations used to solve homographies, fundamental
% matrices etc.
%
% Usage:   [newpts, T] = normalise2dpts(pts)
%
% Argument:
%   pts -  3xN array of 2D homogeneous coordinates
%
% Returns:
%   newpts -  3xN array of transformed 2D homogeneous coordinates
%   T      -  The 3x3 transformation matrix, newpts = T*pts
%           
% Note that if one of the points is at infinity no normalisation
% is possible.  In this case a warning is printed and pts is
% returned as newpts and T is the identity matrix.

% Peter Kovesi
% School of Computer Science & Software Engineering
% The University of Western Australia
% pk@cs.uwa.edu.au    www.cs.uwa.edu.au/~pk
% May 2003


function [newpts, T] = normalise2dpts(pts)

     if ~all(pts(3,:))

       warning('Attempt to normalise a point at infinity')
       newpts = pts;
       T = eye(3);
       return;
     end

     % Ensure homogeneous coords have scale of 1
     pts(1,:) = pts(1,:)./pts(3,:);
     pts(2,:) = pts(2,:)./pts(3,:);

     c = mean(pts(1:2,:)')';      % Centroid.
     newp(1,:) = pts(1,:)-c(1);   % Shift origin to centroid.
     newp(2,:) = pts(2,:)-c(2);

     meandist = mean(sqrt(newp(1,:).^2 + newp(2,:).^2));

     scale = sqrt(2)/meandist;

     T = [scale   0   -scale*c(1)
            0   scale -scale*c(2)
            0     0      1      ];

     newpts = T*pts;


