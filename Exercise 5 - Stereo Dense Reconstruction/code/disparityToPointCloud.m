function [points, intensities] = disparityToPointCloud(...
    disp_img, K, baseline, left_img)
% points should be 3xN and intensities 1xN, where N is the amount of pixels
% which have a valid disparity. I.e., only return points and intensities
% for pixels of left_img which have a valid disparity estimate! The i-th
% intensity should correspond to the i-th point.
    [X,Y] = meshgrid(1:size(disp_img,2),1:size(disp_img,1));
    px_left = [Y(:) X(:) ones(numel(disp_img), 1)]';
    px_right = px_left;
    px_right(2, :) = px_right(2, :) - disp_img(:)';

    % Filter out pixels that do not have a known disparity.
    px_left = px_left(:, disp_img(:)' > 0);
    px_right = px_right(:, disp_img(:)' > 0);
    
    % Switch from (row, col, 1) to (u, v, 1)
    px_left(1:2, :) = flipud(px_left(1:2, :));
    px_right(1:2, :) = flipud(px_right(1:2, :));
    
    % Reproject pixels: Get bearing vectors of rays in camera frame.
    bv_left = K^-1 * px_left;
    bv_right = K^-1 * px_right;

    % Intersect rays according to formula in problem statement.
    points = zeros(size(px_left));
    b = [baseline; 0; 0];
    for i = 1:size(px_left, 2)
        A = [bv_left(:, i) -bv_right(:, i)];
        lambda = (A' * A) \ (A' * b);
        points(:, i) = bv_left(:, i) * lambda(1);
    end
    
    intensities = left_img(disp_img(:)' > 0);
        
end