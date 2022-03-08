%% Part 1: Warping images
I_R = imread('../data/000000.png');
figure(1)

subplot(2,2,1);
imshow(I_R);
title('Reference image');

subplot(2,2,2);
W = getSimWarp(50, -30, 0, 1);
imshow(warpImage(I_R, W), [0, 255]);
title('Translation');

subplot(2,2,3);
W = getSimWarp(0, 0, 10, 1);
imshow(warpImage(I_R, W), [0, 255]);
title('Rotation around upper left corner');

subplot(2,2,4);
W = getSimWarp(0, 0, 0, 0.5);
imshow(warpImage(I_R, W), [0, 255]);
title('Zoom on upper left corner');

%% Part 2: Warped patches and recovering a simple warp with brute force
I_R = imread('../data/000000.png');
figure(2)
% Get and display template:
subplot(1, 2, 1);
W0 = getSimWarp(0, 0, 0, 1);
x_T = [900 291];
r_T = 15;
template = getWarpedPatch(I_R, W0, x_T, r_T);
imagesc(template);
axis equal;
axis off;
title('Template');

subplot(1, 2, 2);
W = getSimWarp(10, 6, 0, 1);
I = warpImage(I_R, W);
r_D = 20;
tic;
[dx, ssds] = trackBruteForce(I_R, I, x_T, r_T, r_D);
toc
imagesc(ssds);
axis equal;
axis off;
title('SSDs');
disp(['Displacement best explained by (dx, dy) = (' num2str(dx) ')']);

%% Part 3: Recovering the warp with KLT
I_R = imread('../data/000000.png');
x_T = [900 291];
r_T = 15;
num_iters = 50;
W = getSimWarp(10, 6, 0, 1);
I = warpImage(I_R, W);
tic;
[W, p_hist] = trackKLT(I_R, I, x_T, r_T, num_iters);
toc
disp(['Point moved by ' num2str(W(:, end)') ', should move by (-10, -6)']);

%% Part 4: Applying KLT to KITTI
I_R = imread('../data/000000.png');
I_R = imresize(I_R, 0.25);
keypoints_rc = load('../data/keypoints.txt') / 4;
keypoints = flipud(keypoints_rc(1:50, :)');
figure(4);
imshow(I_R);
hold on;
plot(keypoints(1, :), keypoints(2, :), 'rx');
hold off;
I_prev = I_R;
pause(0.1);

for i = 1:20
    I = imread(sprintf('../data/%06d.png',i));
    I = imresize(I, 0.25);
    dkp = zeros(size(keypoints));
    parfor j = 1:size(keypoints, 2)
        W = trackKLT(I_prev, I, keypoints(:,j)', r_T, num_iters);
        dkp(:, j) = W(:, end);
    end
    kpold = keypoints;
    keypoints = keypoints + dkp;
    imshow(I);
    hold on;
    plotMatches(1:size(keypoints, 2), flipud(keypoints), flipud(kpold));
    hold off;
    I_prev = I;
    pause(0.1);
end

%% Part 5: Outlier rejection with bidirectional error
I_R = imread('../data/000000.png');
I_R = imresize(I_R, 0.25);
keypoints_rc = load('../data/keypoints.txt') / 4;
keypoints = flipud(keypoints_rc(1:50, :)');
figure(4);
imshow(I_R);
hold on;
plot(keypoints(1, :), keypoints(2, :), 'rx');
hold off;
I_prev = I_R;
pause(0.1);

r_T = 15;
num_iters = 50;
lambda = 0.1;

for i = 1:20
    I = imread(sprintf('../data/%06d.png',i));
    I = imresize(I, 0.25);
    dkp = zeros(size(keypoints));
    keep = true(1, size(keypoints, 2));
    parfor j = 1:size(keypoints, 2)
        [dkp(:,j), keep(j)] = trackKLTRobustly(...
            I_prev, I, keypoints(:,j)', r_T, num_iters, lambda);
    end
    kpold = keypoints(:, keep);
    keypoints = keypoints + dkp;
    keypoints = keypoints(:, keep);
    imshow(I);
    hold on;
    plotMatches(1:size(keypoints, 2), flipud(keypoints), flipud(kpold));
    hold off;
    I_prev = I;
    pause(0.1);
end
