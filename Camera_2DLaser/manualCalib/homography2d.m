% HOMOGRAPHY2D - computes 2D homography
%
% Usage:   H = homography2d(x1, x2)
%
% Arguments:
%          x1  - 3xN set of homogeneous points
%          x2  - 3xN set of homogeneous points such that x1<->x2
% Returns:
%          H - the 3x3 homography such that x2 = H*x1
%
% This code follows the normalised direct linear transformation 
% algorithm given by Hartley and Zisserman p92.
%

% Peter Kovesi
% School of Computer Science & Software Engineering
% The University of Western Australia
% pk@cs.uwa.edu.au    www.cs.uwa.edu.au/~pk
% May 2003

function H = homography2d(x1, x2)

  % check matrix sizes
  if ~all(size(x1) == size(x2))
    error('x1 and x2 must have same dimensions');
  end
  
  % Attempt to normalise each set of points so that the origin 
  % is at centroid and mean distance from origin is sqrt(2).
  [x1, T1] = normalise2dpts(x1);
  [x2, T2] = normalise2dpts(x2);

  % Note that it may have not been possible to normalise
  % the points if one was at infinity so the following does not
  % assume that scale parameter w = 1.

  Npts = length(x1);
  A = zeros(3*Npts,9);

  O = [0 0 0];
  for n = 1:Npts
    X = x1(:,n)';
    x = x2(1,n); y = x2(2,n); w = x2(3,n);
    A(3*n-2,:) = [  O  -w*X  y*X];
    A(3*n-1,:) = [ w*X   O  -x*X];
    A(3*n  ,:) = [-y*X  x*X   O ];
  end

  [U,D,V] = svd(A);

  % Extract homography
  H = reshape(V(:,9),3,3)';

  % Denormalise
  H = T2\H*T1;


