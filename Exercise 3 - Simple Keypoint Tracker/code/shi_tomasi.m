function scores = shi_tomasi(img, patch_size)

sobel_para = [-1 0 1];
sobel_orth = [1 2 1];

Ix = conv2(sobel_orth', sobel_para, img, 'valid');
Iy = conv2(sobel_para', sobel_orth, img, 'valid');

Ixx = double(Ix .^ 2);
Iyy = double(Iy .^ 2);
Ixy = double(Ix .* Iy);

patch = ones(patch_size, patch_size);
pr = floor(patch_size / 2);  % patch radius
sIxx = conv2(Ixx, patch, 'valid');
sIyy = conv2(Iyy, patch, 'valid');
sIxy = conv2(Ixy, patch, 'valid');

scores = 0.5*(sIxx+sIyy-(4*(sIxy.*sIxy)+(sIxx-sIyy).^2).^0.5);
scores(scores<0)=0;

padarray(scores,[1+pr 1+pr]);

