% point_view_matrix.m
%
% Generates the point view matrix based on feature point matches between
% several images.
%
% Input:
%   - matches: cell array of matching feature points
%
% Output:
%   - M: point view matrix
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function M = point_view_matrix(matches)
%% create pointview matrix using matches between image 1 and 2
match_pairs = size(matches, 1);                % number of matched images
M(1,:)      = matches{1,1}(1,inliers);
M(2,:)      = matches{1,1}(2,inliers);


%% add extra columns and rows by adding more images
for i = 2:(match_pairs - 1)
    % find intersection between arrays. IA shows indices of already present
    % matches. IB gives the position of that point in the new matches that
    % need to be added.
    matches = matches{i,1}(:,inliers);
    [~, IA,IB] = intersect(M(i,:),matches(1,:));
    % add next row with matches in next image corresponding to already
    % present points in last column
    M(i+1,IA) = matches(2,IB);
    
    %remove already added matches
    matches(:,IB) = [];
    % add leftover matches to appropriate rows
    M(i:(i+1),(end+1):(end+size(matches,2))) = matches;  
end

orginal = M;
[~, IA,IB] = intersect(M(1,:),M(17,:));
M(:,IA(2:end)) = M(:,IA(2:end)) + M(:,IB(2:end));
M(:,IB(2:end)) = [];
nonzero17 = find(M(17,:));
nonzero1 = find(M(1,:));
checking = ~ismember(nonzero17,nonzero1);
nonzero17 = nonzero17(checking);
tocopy = [M(:,nonzero17)];
M(:,nonzero17) = [];
M = [tocopy M];
M(1,1:size(tocopy,2)) = M(17,1:size(tocopy,2));
M = M(1:16,:); 

end

























