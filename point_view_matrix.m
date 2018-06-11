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
inliers     = matches{1, 2};

M(1:2,:)      = matches{1,1}(:,inliers);

%% add extra columns and rows by adding more images
for i = 2:(match_pairs)
    % find intersection between arrays. IA shows indices of already present
    % matches. IB gives the position of that point in the new matches that
    % need to be added.
    inliers = matches{i, 2};
    match = matches{i,1}(:,inliers);
    [~, IA,IB] = intersect(M(i,:),match(1,:));
    
    % add next row with matches in next image corresponding to already
    % present points in last column
    M(i+1,IA) = match(2,IB);
    
    %remove already added matches
    match(:,IB) = [];
    
    % add leftover matches to appropriate rows
    M(i:(i+1),(end+1):(end+size(match,2))) = match;  
end

% Move matches between first and last frame to the columns assigned to
% points in the first frame
[~, IA,IB] = intersect(M(1,:),M(end,:));
M(:,IA(2:end)) = M(:,IA(2:end)) + M(:,IB(2:end));   % skip index of first match, as these are the first zeros
M(:,IB(2:end)) = [];                                % set copied values to 0 in last columns

% Find matches between first and last frame that did not have a column
% assigned to them
nonzero_last = find(M(end,:));
nonzero_first = find(M(1,:));
no_member = ~ismember(nonzero_last,nonzero_first);          % Check for points not in row one
nonzero_last = nonzero_last(no_member);
tocopy = M(:,nonzero_last);                          % Matches to prepend to row one

% Prepend new matches between first and last image
M(:,nonzero_last) = [];
M = [tocopy M];

% Copy extra row elements to first row, and delete last row
M(1,1:size(tocopy,2)) = M(end,1:size(tocopy,2));
M = M(1:match_pairs,:); 

end

























