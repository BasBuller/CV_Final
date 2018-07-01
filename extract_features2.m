%% Determine sigma, r and c of feature points using normalized Harris corner detection
% extract location of feature points
% Input: 
%   -name: name of the image to extract features from
%   -mode: directly extract from images (1), (0) to read the keypoints
%           from txt file
%
% Output: 
%   -x1, y1, x2, x2: keypoint coordinates in image 1 and 2
%   -desc1, desc2: SIFT descriptors of the keypoints
%
% Requires harris.m
%
% Authors: 
%   -Bas Buller 4166566
%   -Rick Feith 4218272

function [x1 y1 desc1 x2 y2 desc2] = extract_features(name,mode)
%EXTRACT_FEATURES Use extract features using harris or hessian affine
if(mode)
    command = strcat(strcat("/home/rick/Documents/extract_features/extract_features.ln -haraff -i ",name),' -sift -thres 25');
    system(command)
    command = strcat(strcat("/home/rick/Documents/extract_features/extract_features.ln -hesaff -i ",name),' -sift -thres 25');
    system(command)
end

imtxt = strcat(strcat(name),'.haraff.sift');
[x1 y1 ~ ~ ~ desc1] = loadFeatures(imtxt);

imtxt = strcat(strcat(name),'.hesaff.sift');
[x2 y2 ~ ~ ~ desc2] = loadFeatures(imtxt);

end

