clear all
close all
clc

num_scales = 3; % Scales per octave.
num_octaves = 5; % Number of octaves.
sigma = 1.6;
contrast_threshold = 0.04;
image_file_1 = 'images/img_1.jpg';
image_file_2 = 'images/img_2.jpg';
rescale_factor = 0.2; % Rescaling of the original image for speed.

images = {getImage(image_file_1, rescale_factor),...
    getImage(image_file_2, rescale_factor)};

kpt_locations = cell(1, 2);
descriptors = cell(1, 2);

for img_idx = 1:2
    % Write code to compute:
    % 1)    image pyramid. Number of images in the pyarmid equals
    %       'num_octaves'.
    pyramid = build_pyramid(images{img_idx},num_octaves);
    % 2)    blurred images for each octave. Each octave contains
    %       'num_scales + 3' blurred images.
    blurred_img = blur(pyramid,num_scales,sigma);
    % 3)    'num_scales + 2' difference of Gaussians for each octave.
    DoGs = computeDoG(blurred_img);
    % 4)    Compute the keypoints with non-maximum suppression and
    %       discard candidates with the contrast threshold.
    kp_location = get_kp(DoGs,contrast_threshold);
    % 5)    Given the blurred images and keypoints, compute the
    %       descriptors. Discard keypoints/descriptors that are too close
    %       to the boundary of the image. Hence, you will most likely
    %       lose some keypoints that you have computed earlier.
    [descriptors{img_idx},kpt_locations{img_idx}] = computeDescriptors(blurred_img,kp_location,0);

end

% Finally, match the descriptors using the function 'matchFeatures' and
% visualize the matches with the function 'showMatchedFeatures'.
% If you want, you can also implement the matching procedure yourself using
% 'knnsearch'.

indexPairs = matchFeatures(descriptors{1}, descriptors{2},...
    'MatchThreshold', 100, 'MaxRatio', 0.7, 'Unique', true);
% Flip row and column to change to image coordinate system.
% Before         Now
% -----> y       -----> x
% |              |
% |              |
% ⌄              ⌄
% x              y
kpt_matched_1 = fliplr(kpt_locations{1}(indexPairs(:,1), :));
kpt_matched_2 = fliplr(kpt_locations{2}(indexPairs(:,2), :));

figure; ax = axes;
showMatchedFeatures(images{1}, images{2}, kpt_matched_1, kpt_matched_2, ...
    'montage','Parent',ax);
title(ax, 'Candidate point matches');
legend(ax, 'Matched points 1','Matched points 2');


function pyramid = build_pyramid(img,num_octaves)
    pyramid = cell(1, num_octaves);
    pyramid{1} = img;
    for idx = 2:num_octaves
       pyramid{idx} = imresize(pyramid{idx - 1}, 0.5);
    end
end

function blurred_img = blur(pyramid,num_scales,sigma)
    num_octaves = numel(pyramid);
    imgs_per_oct = num_scales+3;
    blurred_img = cell(1,num_octaves);
    for oct_index = 1:num_octaves
        octave_stack = zeros([size(pyramid{oct_index}) imgs_per_oct]);
        for i = 1:imgs_per_oct
            s = i - 2;
            octave_stack(:, :, i) = imgaussfilt(pyramid{oct_index}, sigma * 2^(s / num_scales));
        end
        blurred_img{oct_index} = octave_stack;
    end
end

function DoGs = computeDoG(blurred_img)
    num_octaves = numel(blurred_img);
    DoGs = cell(1, num_octaves);
    for oct_index = 1:num_octaves
        DoG = zeros(size(blurred_img{oct_index})-[0 0 1]);
        num_dog_per_octave = size(DoG,3);
        for i = 1:num_dog_per_octave
            DoG(:,:,i) = abs(blurred_img{oct_index}(:,:,i+1) -blurred_img{oct_index}(:,:,i)); 
        end
        DoGs{oct_index} = DoG;
    end
end

function kp_location = get_kp(DoGs,contrast_threshold)
    num_octaves = numel(DoGs);
    kp_location = cell(1,num_octaves);
    for oct_index=1:num_octaves
        DoG = DoGs{oct_index};
        DoG_max = imdilate(DoG, true(3, 3, 3));
        is_kpt = (DoG == DoG_max) & (DoG >= contrast_threshold);
        is_kpt(:, :, 1) = false;
        is_kpt(:, :, end) = false;
        [x, y, s] = ind2sub(size(is_kpt), find(is_kpt));
        kp_location{oct_index} = [x, y, s];
    end
end


