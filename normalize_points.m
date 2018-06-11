% normalize_points.m
%
% Normalizes feature points (x,y) to zero mean and average distance of
% sqrt(2)
%
% Input:
%   - xi[n, 1]: x coordinates of feature points
%   - yi[n, 1]: y coordinates of feature points
%
% Output:
%   - xn[n, 1]: x coordinates of normalized feature points
%   - yn[n, 1]: y coordinates of normalized feature points
%   - T[3, 3]: transformation matrix for normalization
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function [xn,yn,T] = normalize_points(xi,yi)
mx = 1/max(size(xi)) * sum(xi);
my = 1/max(size(yi)) * sum(yi);
d = 1/max(size(xi))* sum(sqrt((xi-mx).^2 + (yi-my).^2));

T = [sqrt(2)/d 0 -mx*sqrt(2)/d; 0 sqrt(2)/d -my*sqrt(2)/d; 0 0 1];
p = T * [xi; yi; ones(size(xi))];

xn= p(1,:);
yn = p(2,:);

end