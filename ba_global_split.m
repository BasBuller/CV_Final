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

function cost = ba_global_split(X0, point_track)

    load keypoints keypoints

    % Pre assign variables
    cost = 0;
    
    % Split input
    M = X0(:, 1:size(point_track, 1)*2)';
    S = X0(:, (size(point_track, 1)*2+1):end);
    
    % Loop over rows of the sub model
    for i = 1:size(point_track, 1)
        
        % Select camera pose M
        M_loc = M((2*i-1):2*i, :);

        % Select points from model that are visible in current
        % camera S
        [~, cols] = find(point_track(i, :));
        S_loc = S(:, cols);
        
        % Select reference 2D keypoints D_loc
        key_points_cols = point_track(i, cols);
        
        X_loc = keypoints{i, 2};
        Y_loc = keypoints{i, 3};
        D_loc = [X_loc; Y_loc];
        D_loc = D_loc(:, key_points_cols);

        % Calculate error  
        E = D_loc - M_loc*S_loc;
        E = sqrt(E(1,:).^2 + E(2,:).^2);
        cost = cost + sum(E);
    end
end