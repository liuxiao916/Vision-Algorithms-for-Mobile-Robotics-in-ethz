function keypoints = selectKeypoints(scores, num, r)
% Selects the num best scores as keypoints and performs non-maximum 
% supression of a (2r + 1)*(2r + 1) box around the current maximum.

keypoints = zeros(2,num);

temp = padarray(scores,[r r]);

for i =1:num
    [~, position]=max(temp(:));
    [row, col] = ind2sub(size(temp), position);
    keypoints(:,i)=[row-r;col-r];
    temp(row-r:row+r, col-r:col+r) = zeros(2*r + 1, 2*r + 1);

end

end