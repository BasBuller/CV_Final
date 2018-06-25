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

function cost = bundle_adjustment(complete_model)
    load triple_order triple_order
    load triple_models triple_models
    
    % Pre assign variables
    cost = 0;
    prev_ind = 0;
    
    % Build Motion matrix for the complete model, adhere to the order
    % of triple view models used during procrustes analysis
    for i = triple_order
        X_true = triple_models{i, 4};
        M = triple_models{i, 5};
        num_points = max(size(X_true));
        
        local_model = complete_model(:, prev_ind + 1: prev_ind + num_points);
        prev_ind = prev_ind + num_points;
        diff = (X_true - M * local_model).^2;
        cost = cost + sum(sum(diff));
    end
end