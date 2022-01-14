function descriptors = describeKeypoints(img, keypoints, r)
% Returns a (2r+1)^2xN matrix of image patch vectors based on image
% img and a 2xN matrix containing the keypoint coordinates.
% r is the patch "radius".
[~,keypoints_number] = size(keypoints);
descriptors = zeros((2*r+1)^2,keypoints_number);
pad_img = padarray(img, [r, r]);

for i=1:keypoints_number
    descriptors(:,i) = reshape(pad_img(keypoints(1,i):keypoints(1,i)+2*r,keypoints(2,i):keypoints(2,i)+2*r),[],1);
end

end