% loadfeatures.m
%
% Loads the features from a SIFT feature txt file
%
% Input:
%   - file: filename of the txt file
%
% Output:
%   - x, y: coordinates of the keypoints
%   - a, b, c: scaling and orientation
%   - desc: SIFT descriptor
%   - nb: name of the image
%   - dim: dimension of the descriptor
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function [x y a b c  desc nb dim]=loadFeatures(file)
fid = fopen(file, 'r');
dim=fscanf(fid, '%f',1);
if dim==1
dim=0;
end

% scan and open feature file
nb=fscanf(fid, '%d',1);
feat = fscanf(fid, '%f', [5+dim, inf]);
fclose(fid);
feat = feat';

% load features into separate parameters
x = feat(:,1);
y = feat(:,2);
a = feat(:,3);
b = feat(:,4);
c = feat(:,5);
desc = feat(:,6:end);
end