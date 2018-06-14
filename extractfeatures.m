%% Determine sigma, r and c of feature points using normalized Harris corner detection
% extract location of feature points
% Input: 
%   -img: imagename
%
% Output: 
%   -r,c: row and column of cornerpoints
%   -s: sigma corresponding to the cornerpoints
%
% Requires harris.m
%
% Authors: 
%   -Bas Buller 4166566
%   -Rick Feith 4218272

%%normalize harris function
function [s,r,c] = normalizedHarris(img,loops,threshold)
%load image in grayscale
img = (rgb2gray(imread(img)));

%% loop over different sigma values and store r and c

sizeIm = size(img);
% Create mapping to store all cornerpoints at each scale
Ls = zeros(sizeIm(1),sizeIm(2),loops);
finalImage = zeros(size(img));

sigmarange = [1.2.^(-9:1:(loops-10))]; % loop 1: 1.2^-4 .... loop n: 1.2^(loop-5). Low sigma values yield more feature points.
for i = 1:1:loops
    li = zeros(sizeIm);
    sigma = sigmarange(i);
    % detect cornerpoints for different sigmas
    [r,c] = harris(img,sigma,threshold) ;
%     fprintf(strcat("harris loop ",num2str(i)," completed \n"))
    % attach value to cornerpoints
    Laplacian = imfilter(img, fspecial('log',[3 3],sigma),'replicate','same') .* (sigma^2);
    li(sub2ind(sizeIm,r,c)) = abs(Laplacian(sub2ind(sizeIm,r,c)));
    Ls(:,:,i)=li;
end

% check which scale belongs to the cornerpoints by looking for the maximum
for i = 1:1:loops
    finalImage = max(Ls(:,:,i),finalImage);
end

% ensure each cornerpoints is only detected once
finalBlock = imdilate(finalImage,strel('square',3));
finalClean = (finalBlock == finalImage)& (finalImage>0);

%store sigmas belong to cornerpoints
s = zeros(size(img));
for i = 1:1:loops
    sigma = sigmarange(i);
    s(finalBlock==Ls(:,:,i)&(Ls(:,:,i)~=0))=sigma;
end


 [r,c] = find(finalClean);
end