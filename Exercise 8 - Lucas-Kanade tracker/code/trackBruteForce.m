function [dx, ssds] = trackBruteForce(I_R, I, x_T, r_T, r_D)
% I_R: reference image, I: image to track point in, x_T: point to track,
% expressed as [x y]=[col row], r_T: radius of patch to track, r_D: radius
% of patch to search dx within; dx: translation that best explains where
% x_T is in image I, ssds: SSDs for all values of dx within the patch
% defined by center x_T and radius r_D.
ssds = zeros(2*r_D+1);

template = getWarpedPatch(I_R, getSimWarp(0,0,0,1), x_T, r_T);
for dx = -r_D:r_D
    for dy = -r_D:r_D
        candidate = getWarpedPatch(I, getSimWarp(dx,dy,0,1), x_T, r_T);
        ssd = sum(sum((template - candidate).^2));
        ssds(dx + r_D + 1, dy + r_D + 1) = ssd;
    end
end

[~, idx] = min(ssds(:));
[dx1, dx2] = ind2sub(size(ssds), idx);
dx = [dx1 dx2] - r_D - 1;

end