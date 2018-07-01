% gaussianDer.m
%
% Determines the derivative of a Gaussian filter
%
% Input:
%   - sigma: value of the standard deviation
%
% Output:
%   - Gd: Derivative of Gaussian filter
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function Gd = gaussianDer(sigma)
x = -3.*sigma:3.*sigma;
Gd = -x./(sigma.^2).*gaussian(sigma);
end