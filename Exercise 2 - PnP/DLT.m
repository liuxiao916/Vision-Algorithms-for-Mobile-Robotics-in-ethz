%The DLT algorithm
%Author: Xiao Liu
clear,clc;

%Load_data
img1 = imread('data\images_undistorted\img_0001.jpg');
img1_gray = rgb2gray(img1);
corners=load('data\detected_corners.txt');
K = load('data\K.txt');
corners_world = load('data\p_W_corners.txt')*0.01;
number_of_points = size(corners_world,1);

%Derivation of the linear system of equations to solve
%normalized_corners=inv(K)*[reshape(corners(1,:),2,[]);ones([number_of_points,1])'];
%Q = zeros([2*number_of_points,12]);

%for index = 1:number_of_points
%    Q(2*index-1,:)=[corners_world(index,1),corners_world(index,2),corners_world(index,3),1,0,0,0,0,-normalized_corners(1,index)*corners_world(index,1),-normalized_corners(1,index)*corners_world(index,2),-normalized_corners(1,index)*corners_world(index,3),-normalized_corners(1,index)];
%    Q(2*index,:)=[0,0,0,0,corners_world(index,1),corners_world(index,2),corners_world(index,3),1,-normalized_corners(2,index)*corners_world(index,1),-normalized_corners(2,index)*corners_world(index,2),-normalized_corners(2,index)*corners_world(index,3),-normalized_corners(2,index)];
%end
M = estimatePoseDLT(corners, corners_world, K) 
projected_points = reprojectPoints([corners_world';ones(1,12)] ,M,K)

%plot
figure(1)
imshow(img1_gray)
hold on
% plot(projected_points(1,:),projected_points(2,:),'.','MarkerSize',20,'color',[1,0,0])
corners_reshape = reshape(corners(1,:),2,[])
plot(corners_reshape(1,:), corners_reshape(2,:), 'o'); 
hold on;
plot(projected_points(1,:),projected_points(2,:), '+');
legend('Original points','Reprojected points');

hold off


function M = estimatePoseDLT(p, P, K) 
    number_of_points = size(P,1);
    normalized_p=inv(K)*[reshape(p(1,:),2,[]);ones([number_of_points,1])'];
    Q = zeros([2*number_of_points,12]);

    for index = 1:number_of_points
        Q(2*index-1,:)=[P(index,1),P(index,2),P(index,3),1,0,0,0,0,-normalized_p(1,index)*P(index,1),-normalized_p(1,index)*P(index,2),-normalized_p(1,index)*P(index,3),-normalized_p(1,index)];
        Q(2*index,:)=[0,0,0,0,P(index,1),P(index,2),P(index,3),1,-normalized_p(2,index)*P(index,1),-normalized_p(2,index)*P(index,2),-normalized_p(2,index)*P(index,3),-normalized_p(2,index)];
    end

    [U,S,V]=svd(Q);
    M = reshape(V(:,12),[4,3])';
end 

function projected_points =  reprojectPoints(points,M,K)
    T = M;
    if M(12)<0
        M = -M;
    end
    R_hat = M(1:3,1:3);
    [U,S,V] = svd(R_hat);
    R = U*V';
    alpha = norm(R_hat,'fro')/norm(R,'fro');
    t = M(1:3,4)/alpha;

    T(1:3,1:3)=R;
    T(1:3,4)=t;
    projected_points = K*T*points;
    projected_points = projected_points./projected_points(3,:);
end
