function [descriptors, final_kpt_locations] = ...
    computeDescriptors(blurred_images, kpt_locations, rot_invariant)
num_octaves = numel(blurred_images);
assert(num_octaves == numel(kpt_locations));

descriptors = {};
final_kpt_locations = {};

% The magic multiplication of 1.5 taken from Lowe's paper.
gausswindow = fspecial('gaussian', [16, 16], 16 * 1.5);
for oct_idx = 1:num_octaves
    oct_blurred_imgs = blurred_images{oct_idx};
    oct_kpt_locations = kpt_locations{oct_idx};
    % Only go through relevant images.
    relevant_img_indices = unique(oct_kpt_locations(:, 3));
    for img_idx = relevant_img_indices'
        curr_image = oct_blurred_imgs(:, :, img_idx);
        [rows_img, cols_img] = size(curr_image );
        [Gmag, Gdir] = imgradient(curr_image );
        
        is_kpt_in_image = oct_kpt_locations(:, 3) == img_idx;
        img_kpt_locations = oct_kpt_locations(is_kpt_in_image, :);
        img_kpt_locations = img_kpt_locations(:, 1:2);
        
        num_kpts = size(img_kpt_locations, 1);
        img_descriptors = zeros(num_kpts, 128);
        is_valid = false(num_kpts, 1);
        for corner_idx = 1:num_kpts
            row = img_kpt_locations(corner_idx, 1);
            col = img_kpt_locations(corner_idx, 2);
            if row > 8 && col > 8 && row < rows_img - 7 && ...
                    col < cols_img - 7
                is_valid(corner_idx) = true;
                Gmag_loc = Gmag(row-8:row+7, col-8:col+7);
                Gmag_loc_w = Gmag_loc.*gausswindow;
                Gdir_loc = Gdir(row-8:row+7, col-8:col+7);
                
                Gmag_loc_derotated_w = Gmag_loc_w;
                Gdir_loc_derotated = Gdir_loc;
                
                if rot_invariant == 1
                    % compute dominant direction through looking at the most
                    % common orientation in the histogram, spaced at 10 deg
                    angle_edges = -180:10:180;
                    orient_hist = weightedhistc(Gdir_loc(:), Gmag_loc_w(:), angle_edges);
                    [~, max_orient_idx] = max(orient_hist);
                    Gdir_loc_principal = (angle_edges(max_orient_idx) +...
                    angle_edges(max_orient_idx + 1)) / 2;
                    
                    % derotate patch
                    patch_derotated = derotatePatch(curr_image, [row, col], 16, Gdir_loc_principal);
                    [Gmag_loc_derotated, Gdir_loc_derotated] = imgradient(patch_derotated);
                    Gmag_loc_derotated_w = Gmag_loc_derotated * gausswindow;
                end
                
                N_tmp = 1;
                for ix=1:1:4
                    for iy=1:1:4
                        N_w = weightedhistc(...
                            reshape(Gdir_loc_derotated(4*ix-3:4*ix,4*iy-3:4*iy),1,16)...
                            , reshape(Gmag_loc_derotated_w(4*ix-3:4*ix,4*iy-3:4*iy),1,16)...
                            , -180:45:180);
                        img_descriptors(corner_idx , N_tmp:N_tmp+7) = N_w(1, 1:8);
                        N_tmp = N_tmp + 8;
                    end
                end
            end
        end
        % Adapt keypoint location such that they correspond to the
        % originial image dimensions.
        img_kpt_locations = img_kpt_locations.*2^(oct_idx - 1);
        % Only store valid keypoints.
        descriptors{end+1} = img_descriptors(is_valid, :);
        final_kpt_locations{end+1} = img_kpt_locations(is_valid, :);
    end
end
% Normalize the descriptors such that they have unit norm.
descriptors = normalize(cell2mat(descriptors'), 2, 'norm', 2);
final_kpt_locations = cell2mat(final_kpt_locations');
end
