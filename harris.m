% harris.m
%
% This function finds Harris corners x and y coordinates at integration-scale sigma.
%
% Input: 
%   -im: imagename
%   -sigma: sigma integration-scale
%
% Output: 
%   -r,c: row and column of feature point location
%
% Requires ImageDerivatives, gaussianDer. gaussian
%
% Authors: 
%   -Bas Buller 4166566
%   -Rick Feith 4218272

function [r, c] = harris(im, sigma,Rthreshold)

img = im;
gamma = 0.7; % The derivative-scale is gamma times the integration scale

% Calculate Gaussian Derivatives at derivative-scale
Ix =  ImageDerivatives(img,gamma*sigma, gamma*sigma, "x") * gamma*sigma;
Iy = ImageDerivatives(img,gamma*sigma, gamma*sigma, "y") * gamma*sigma;

% Allocate a 3-channel image to hold the 3 parameters for each pixel
M = zeros(size(Ix,1), size(Ix,2), 3);

% Calculate M for each pixel
M(:,:,1) = Ix.^2;
M(:,:,2) = Ix.*Iy;
M(:,:,3) = Iy.^2;

% Smooth M with a gaussian at the integration scale sigma.
M = imfilter(M, fspecial('gaussian', ceil(sigma*6+1), sigma), 'replicate', 'same');

% Compute the cornerness R
detM = M(:,:,1).*M(:,:,3)-(M(:,:,2).^2);
traceM = M(:,:,1)+M(:,:,3);
k = 0.05;
R =  detM - k*(traceM.^2);

% Set the threshold 
threshold = Rthreshold * max(max(R));

% Find local maxima
% Dilation will alter every pixel except local maxima in a 3x3 square area.
% Also checks if R is above threshold
R = ((R>threshold) & ((imdilate(R, strel('square', 3))==R)));%.* sigma;

% Return the coordinates
[r,c] = find(R);
end