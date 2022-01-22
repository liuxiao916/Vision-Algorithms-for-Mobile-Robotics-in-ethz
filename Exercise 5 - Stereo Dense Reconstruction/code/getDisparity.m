function disp_img = getDisparity(left_img, right_img, patch_radius, min_disp, max_disp)
% left_img and right_img are both H x W and you should return a H x W
% matrix containing the disparity d for each pixel of left_img. Set
% disp_img to 0 for pixels where the SSD and/or d is not defined, and for d
% estimates rejected in Part 2. patch_radius specifies the SSD patch and
% each valid d should satisfy min_disp <= d <= max_disp.

    refinement = false;
    outlier_removal = true;

    [rows,cols] = size(left_img);
    disp_img=zeros(rows,cols);
    
    col_start = patch_radius+max_disp+1;
    col_end = cols-patch_radius;

    parfor row = (patch_radius+1):(rows-patch_radius)
        for col = col_start:col_end
            left_patch = single(left_img((row-patch_radius):(row+patch_radius),(col-patch_radius):(col+patch_radius)));
            left_vec = left_patch(:);
            right_vec = single(zeros((2*patch_radius+1)*(2*patch_radius+1),max_disp-min_disp+1));
            for i=min_disp:max_disp
                right_patch = single(right_img((row-patch_radius):(row+patch_radius),(col-patch_radius-i):(col+patch_radius-i)));
                right_vec(:,max_disp-i+1) = right_patch(:);
            end
            ssds = pdist2(left_vec', right_vec', 'squaredeuclidean');
            [min_ssd, disp_index] = min(ssds);


            %step 2 outlier removal
            if outlier_removal
                if (nnz(ssds <= 1.5 * min_ssd) < 3 && disp_index ~= 1 && disp_index ~= length(ssds))
                    if refinement                   
                        x = [disp_index-1 disp_index disp_index+1];
                        p = polyfit(x, ssds(x), 2);
                        disp_img(row, col) = max_disp +1+ p(2)/(2*p(1));
                    else
                        disp_img(row,col)=max_disp-disp_index+1;
                    end
                end
            else
                disp_img(row,col)=max_disp-disp_index+1;
            end

        end
    end



