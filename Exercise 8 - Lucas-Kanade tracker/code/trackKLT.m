function [W, p_hist] = trackKLT(I_R, I, x_T, r_T, num_iters)
% I_R: reference image, I: image to track point in, x_T: point to track,
% expressed as [x y]=[col row], r_T: radius of patch to track, num_iters:
% amount of iterations to run; W(2x3): final W estimate, p_hist 
% (6x(num_iters+1)): history of p estimates, including the initial
% (identity) estimate

p_hist = zeros(6, num_iters+1);
W = getSimWarp(0, 0, 0, 1); % identity
p_hist(:, 1) = W(:); % p is just a reshape of W

% T suffix indicates image evaluated for patch T
I_RT = getWarpedPatch(I_R, W, x_T, r_T); % patch of reference image
i_R = I_RT(:); % the template vector i_R never changes.

% x and y coordinates of the patch also never change
xs = -r_T:r_T;
ys = -r_T:r_T;
n = numel(xs);
xy1 = [kron(xs, ones([1 n]))' kron(ones([1 n]), ys)' ones([n*n 1])];
dwdx = kron(xy1, eye(2));

do_plot = true;

if do_plot
    figure(3);
end

for iter = 1:num_iters
    % Getting more, for a valid convolution.
    big_IWT = getWarpedPatch(I, W, x_T, r_T + 1);
    IWT = big_IWT(2:end-1, 2:end-1);
    i = IWT(:);
    
    % getting di/dp
    IWTx = conv2(1, [1 0 -1], big_IWT(2:end-1, :), 'valid');
    IWTy = conv2([1 0 -1], 1, big_IWT(:, 2:end-1), 'valid');
    didw = [IWTx(:) IWTy(:)]; % as written in the statement
    didp = zeros(n*n, 6);
    for pixel_i = 1:n*n
        didp(pixel_i, :) = didw(pixel_i, :) * ...
            dwdx(pixel_i*2-1:pixel_i*2, :);
    end
    
    % Hessian
    H = didp' * didp;
    
    % Putting it together and incrementing
    delta_p = H^-1 * didp' * (i_R - i);
    W = W + reshape(delta_p, [2 3]);
        
    if do_plot
        subplot(3, 1, 1);
        imagesc([IWT I_RT (I_RT - IWT)]);
        title('I(W(T)), I_R(T) and the difference')
        colorbar;
        axis equal;
        subplot(3, 1, 2);
        imagesc([IWTx IWTy]);
        title('warped gradients')
        colorbar;
        axis equal;
        subplot(3, 1, 3);
        descentcat = zeros(n, 6*n);
        for j = 1:6
            descentcat(:, (j-1)*n+1:j*n) = reshape(didp(:, j), [n n]);
        end
        imagesc(descentcat);
        title('steepest descent images');
        colorbar;
        axis equal;
        pause(0.1)
    end
    
    p_hist(:, iter + 1) = W(:);
    
    if norm(delta_p) < 1e-3
       p_hist = p_hist(:, 1:iter+1);
       return
    end
end

end