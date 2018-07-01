% sift_descriptor.m
%
% extract location of feature points using vl_sift
%
% Input: 
%   -name: imagename
%   -r,c: row column corresponding to corner points
%   -s: sigma value corresponding to corner points
%
% Output: 
%   -x,y: x and y location of cornerpoint
%   -d: sift descriptor of image
%
% Requires harris.m
%
% Authors: 
%   -Bas Buller 4166566
%   -Rick Feith 4218272

function [x,y,d] = sift_descriptor(name, s, r, c)

% create list of sigma values
s_values = s(sub2ind(size(s),r,c));

% run vl_sift
[f,d] = vl_sift(single(rgb2gray(imread(name))),'frames',[[c'];[r'];[2.*s_values+1]';[zeros(size(s_values))]'],'orientations');
x = f(1,:);
y = f(2,:);

end