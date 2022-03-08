function patch = getWarpedPatch(I, W, x_T, r_T)
% x_T is 1x2 and contains [x_T y_T] as defined in the statement. patch is
% (2*r_T+1)x(2*r_T+1) and arranged consistently with the input image I.

patch = zeros(2*r_T+1);

interpolate = true;

% Somehow pre-calculating this resulted in major speedup.
max_coords = fliplr(size(I));
WT = W';

for x = -r_T:r_T
    for y = -r_T:r_T
        pre_warp = [x y];
        warped = x_T + [pre_warp 1] * WT;
        if all(warped < max_coords & warped > [1 1])
            % Don't forget that patch coefficients are 1:(2*r_T+1), rather
            % than -r_T:r_T.
            if interpolate
                floors = floor(warped);
                weights = warped - floors;
                a = weights(1); b = weights(2);            
                intensity = (1-b) * (...
                    (1-a) * I(floors(2), floors(1)) +...
                    a * I(floors(2), floors(1)+1))...
                    + b * (...
                    (1-a) * I(floors(2)+1, floors(1)) +...
                    a * I(floors(2)+1, floors(1)+1));
                patch(y + r_T + 1, x + r_T + 1) = ...
                    intensity;
            else
                patch(y + r_T + 1, x + r_T + 1) = ...
                    I(int32(warped(2)), int32(warped(1)));
            end
        end
    end
end

end