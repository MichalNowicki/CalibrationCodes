function circ = great_circ(v1,v2);
%
% generate 40 3D-points of great circle along plane v1 v2
% ... normal to vector v1 X v2
%
%
% (c) jlobo Set2003


switch nargin   % 1 arg => nromal vector; 2 args => 2 points

case 2  %2 args => 2 points
    
n=40;
circ=[];
% Gram-Schmidt
v1=unit(v1);
v2=v2-dot(v1,v2)*v1;
v2=unit(v2);
for i = 0:n
    theta=i/n*2*pi;
    a=cos(theta);
    b=sin(theta);
    m=a*v1+b*v2;
    m=unit(m);
    circ=[circ; m];
end


case 1

n=40;
circ=[];
% vectors normal to given v1
nv=v1;
v1(1)=-nv(3)/nv(1);
v1(2)=0;
v1(3)=1;
v2(1)=0;
v2(2)=1;
v2(3)=-nv(2)/nv(3);
v2=unit(v2);
% Gram-Schmidt
v1=unit(v1);
v2=v2-dot(v1,v2)*v1;
v2=unit(v2);
for i = 0:n
    theta=i/n*2*pi;
    a=cos(theta);
    b=sin(theta);
    m=a*v1+b*v2;
    m=unit(m);
    circ=[circ; m];
end
    
    
otherwise
end    

%
