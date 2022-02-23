function P = linearTriangulation(p1,p2,M1,M2)
% LINEARTRIANGULATION  Linear Triangulation
%
% Input:
%  - p1(3,N): homogeneous coordinates of points in image 1
%  - p2(3,N): homogeneous coordinates of points in image 2
%  - M1(3,4): projection matrix corresponding to first image
%  - M2(3,4): projection matrix corresponding to second image
%
% Output:
%  - P(4,N): homogeneous coordinates of 3-D points
[height,width] = size(p1);
P = zeros(4,width);

for i = 1:width
    A = [cross2Matrix(p1(:,i))*M1;cross2Matrix(p2(:,i))*M2];
    [U,S,V]=svd(A);
    P(:,i) = V(:,4)/V(4,4);
end

return