function W = getSimWarp(dx, dy, alpha_deg, lambda)
% alpha given in degrees, as indicated
theta = alpha_deg/180*pi;
W = lambda*([cos(theta),-sin(theta),dx;sin(theta),cos(theta),dy]);
