%% Determine matching cornerpoints between 2 images
% Input: 
%   -x1,y1,x2,y2: x and y coordinates belonging to the cornerpoints of
%   image 1 and 2
%   -d1,d2: sift descriptors of image 1 and 2
%   -nn_threshold: threshold for determining if match is correctusing
%   nearest neighbour 
%
% Output: 
%   -match: index of matching cornerpoint in image 1 and 2
%   
%
% Requires harris.m
%
% Authors: 
%   -Bas Buller 4166566
%   -Rick Feith 4218272

function [match] = match_features(xa,ya,da,xb,yb,db,nn_threshold)
da = double(da);
db= double(db);

% determine euclidian distance between each descriptor
da2 = (sum(da.^2,1) .* ones(size(db,2),size(da,2)))';
db2 = sum(db.^2,1) .* ones(size(da,2),size(db,2));
dab = da'*db;
dist = sqrt(da2 + db2 - 2.*dab);

% Find minimum distances and nearest neighbour 
[min_dist,min_dist_ind] = mink(dist,2,2);
distR = min_dist(:,1)./min_dist(:,2);

% check if nearest neighbour above threshold
distR(distR>0.8) = 0;
match1 = find(distR);
match2 = min_dist_ind(distR>0,1);
match = [match1';match2'];
end