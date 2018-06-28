% ba_global.m
%
% Applies bundle adjustment to the complete 3D point model 
%
% Input:
%   -   X0 = [M local model]
%   -   key_pts = coordinates of the initial keypoints in the camera
%   -   view_order = order in which the three view models are stitched
%                    together
% Output:
%   -   cost = summed and squared deviation between projecten and real
%              keypoints
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function cost = ba_global(X0)
    load triple_models triple_models
    load updated_triple_models updated_triple_models
    load triple_order triple_order
    
    % Pre assign variables
    cost = 0;
    prev_ind = 0;
    
    % Split input
    M = X0(:, 1:max(size(triple_order))*2)';
    complete_model = X0(:, (max(size(triple_order))*2+1):end);
    
    % Perform 2 loops, outer loop over the different appended 3D models,
    % inner loop over the rows corresponding to a camera view
    for i = triple_order        
        % Skip empty models
        if (size(updated_triple_models{i, 1}))
            n_pts = max(size(updated_triple_models{i, 1}));
            point_track = triple_models{i, 6};
            
            % Loop over rows of the sub model
            for j = 1:size(point_track, 1)
                
                % Select camera pose M
                M_loc = M(2*j-1:2*j, :);
                
                % Select points from model that are visible in current
                % camera S
                S_loc = complete_model(:, (prev_ind + 1):(prev_ind + n_pts));
                [~, cols] = find(point_track(j, :));
                S_loc = S_loc(:, cols);
                
                % Select keypoints corresponding to D (= M*S)
                D_loc = triple_models{i, 4}(1:2, cols);
                
                % Calculate error
                E = D_loc - M_loc*S_loc;
                E = sqrt(E(1,:).^2 + E(2,:).^2);
                cost = cost + sum(E);
            end
            
            prev_ind = prev_ind + n_pts;
        else
            continue;
        end
    end
end