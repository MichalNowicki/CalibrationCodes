% vect_ang
%
% angle between two vectors
%
% angle=vect_ang(v1,v2)
%
% jlobo Set 2003

function angle=vect_ang(v1,v2)

angle=acos(dot(v1,v2)/(norm(v1)*norm(v2)));





