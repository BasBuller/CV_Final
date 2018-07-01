% gaussian.m
%
% Determines a Gaussian filter
%
% Input:
%   - sigma: value of the standard deviation
%
% Output:
%   - G: Gaussian filter
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function G = gaussian(sigma)
x = -3.*sigma : 3.*sigma ;

g = 1./(sigma.*(2*pi)^(1/2))*exp(-(x.^2)./(2.*sigma.^2));
G = g./sum(g);
end