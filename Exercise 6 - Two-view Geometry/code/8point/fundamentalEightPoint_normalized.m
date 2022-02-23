function F = fundamentalEightPoint_normalized(p1, p2)
% estimateEssentialMatrix_normalized: estimates the essential matrix
% given matching point coordinates, and the camera calibration K
%
% Input: point correspondences
%  - p1(3,N): homogeneous coordinates of 2-D points in image 1
%  - p2(3,N): homogeneous coordinates of 2-D points in image 2
%
% Output:
%  - F(3,3) : fundamental matrix
%

[p1_hat,T1]=normalise2dpts(p1);
[p2_hat,T2]=normalise2dpts(p2);

[height, width]=size(p1);
Q = zeros(width,9);

for i = 1:width
    Q(i,:)=kron(p1_hat(:,i),p2_hat(:,i));
end

[U,S,V]=svd(Q);

vec_F = V(:,9);
[U,S,V] = svd(reshape(vec_F,[3,3]));

S(3,3)=0;
F_hat = U*S*V';

F = T2'*F_hat*T1;


