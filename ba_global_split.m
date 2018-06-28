% ba_global.m
%
% Applies bundle adjustment to the complete 3D point model 
%
% Input:
%   -   X0 = [M local model]
%   -   point_track = boolean matrix of points visible per camera
%   -   key_pts = coordinates of the local keypoints
%
% Output:
%   -   cost = summed and squared deviation between projecten and real
%              keypoints
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function cost = ba_global_split(X0, point_track, D_loc)

    % Pre assign variables
    cost = 0;
    
    % Split input
    M = X0(:, 1:size(point_track, 1)*2)';
    S_loc = X0(:, (size(point_track)*2+1):end);
    
    % Loop over rows of the sub model
    for j = 1:size(point_track, 1)

        % Select camera pose M
        M_loc = M(2*j-1:2*j, :);

        % Select points from model that are visible in current
        % camera S
        [~, cols] = find(point_track(j, :));
        S_loc = S_loc(:, cols);
        
        % Setting up the correct D matrix is still somewhat hard
        size(D_loc)
        size(M_loc)
        size(S_loc)

        % Calculate error  
        E = D_loc - M_loc*S_loc;
        E = sqrt(E(1,:).^2 + E(2,:).^2);
        cost = cost + sum(E);
    end
end