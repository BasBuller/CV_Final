% ba_local.m
%
% Applies local bundle adjustment to the partial 3D point models
%
% Input:
%   -   X0 = [M local model]
%   -   key_pts = coordinates of the initial keypoints in the camera
%   -   type = if the model consists of three of four views
%              plane
%
% Output:
%   -   cost = summed and squared deviation between projecten and real
%              keypoints
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function cost = ba_local(X0, key_pts, type)
    % Split inputs
    if (type == 3)
        M = X0(:, 1:6)';
        S = X0(:, 7:end);
    else
        M = X0(:, 1:8)';
        S = X0(:, 9:end);
    end
        
    % Cost
    cost = 0;
    error = key_pts - M*S;
    for i = 1:type
        cost = cost + sum(sqrt(error((2*i-1), :).^2 + error(2*i, :).^2));
    end
end