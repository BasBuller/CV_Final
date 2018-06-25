% bundle_adjustment_complete.m
%
% Applies bundle adjustment to the complete 3D point model 
%
% Input:
%   -   model = complete 3D point model
%   -   pvm = point view matrix
%   -   keypoints = coordinates of the initial keypoints in the camera
%   plane
%   -   M = Motion matrix from the Structure from Motion
%
% Output:
%   -   adj_model = adjusted 3D point model
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function cost = bundle_adjustment(X0)
    load triple_order triple_order
    load triple_models triple_models
    load updated_triple_models updated_triple_models
    
    % Pre assign variables
    cost = 0;
    prev_ind = 0;
    
    % Split input
    M = X0(:, 1:max(size(triple_order))*6)';
    complete_model = X0(:, (max(size(triple_order))+1):end);
    
    % Per three view model multiply corresponding motion matrix with 3D model keypoints
    for i = triple_order
        % First weave original keypoint coordinates into x-y-x-y
        if (max(size(updated_triple_models{i, 1})) > 0)
            orig_keyp = triple_models{i, 4};
        else 
            % If the three view model was not appended to the complete
            % model skip the current iteration
            continue
        end
        
        % Select M matrix and 3D model points corresponding to triple_model
        % i
        pts_step = max(size(updated_triple_models{i, 1}));
        Mloc = M(((i-1)*6 + 1):i*6, :);
        local_model = complete_model(:, (prev_ind + 1): (prev_ind + pts_step));
        
        % Update prev_ind for following loop
        prev_ind = prev_ind + pts_step;
        
        % Compare projected 2 coordintes with original, measured keypoints
        loc_cost = orig_keyp - Mloc * local_model;
        upd_cost = sqrt([loc_cost(1,:).^2 + loc_cost(2,:).^2; loc_cost(3,:).^2 + loc_cost(4,:).^2; loc_cost(5,:).^2 + loc_cost(6,:).^2]);
        cost = sum(sum(upd_cost));
    end
end