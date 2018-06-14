% model_stitching.m
%
% Combines all the 3D coordinate models of three and four consecutive
% images
%
% Input:
%   -   triple_models = 3D coordinates from three consecutive images
%   -   quad_models = 3D coordinates from four consecutive images
%
% Output:
%   -   complete_model = complete 3D model
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function complete_model = model_stitching(triple_models, quad_models)
    % Find biggest four view model, set as starting point
    ind = 0;
    len = 0;
    for i = 1:max(size(quad_models))
        if(max(size(quad_models{i})) > len)
            len = max(size(quad_models{i,1}));
            ind = i;
        end
    end
    
    % Determine order of matching models
    order = [ind:max(size(quad_models)) 1:ind];
    
    % Preassign cell array for uodated models
    updated_triple_models = cell(max(size(triple_models)), 1);
    updated_quad_models = cell(max(size(quad_models)), 1);
    
    
%% Update first two views
    % First quad view model is not transformed
    updated_quad_models(ind) = {quad_models{ind, 1}};
    
    % Assign temporary working variables
    if(ind == length(triple_models))
        triple = triple_models{1, 1};
    else
        triple = triple_models{(ind+1), 1};
    end
    quad = quad_models{ind, 1};
        
    % Find matching points between three and four view models
    if(ind == length(triple_models))
        match_triple = triple_models{1, 2};
    else
        match_triple = triple_models{(ind+1), 2};
    end        
    match_quad = quad_models{ind, 2}(2:4, :);
        
    % Find intersection of points between four and three view
    [~, ~, IB] = intersect(match_quad', match_triple', 'rows', 'stable');
        
    % Procrustes with matching points between three and four view
    [~, ~, trans] = procrustes(quad', triple(:, IB)');
        
    % Apply transform to entire three view model and save in cell array
    new_triple = trans.b * triple' * trans.T + trans.c(1, :);
    if(ind == length(triple_models))
        updated_triple_models(1) = {new_triple'};
    else
        updated_triple_models(ind+1) = {new_triple'};
    end
    
    % Final, complete 3D point model
    complete_model = [new_triple'];
%     complete_model = [quad_models{ind, 1} new_triple'];
    
    
%% Loop over remaining views
    % Loop over models to determine procrustes transforms, first fit three
    % view to four view, next fir four view to three view.
    for i = 2:length(order)-1
        % Assign temporary working variables
        triple = updated_triple_models{order(i),1};
        quad = quad_models{order(i),1};
        
        match_triple = triple_models{order(i), 2};
        match_quad = quad_models{order(i), 2}(1:3, :);
        
        % Find intersection of points between four and three view
        [~, ~, IB] = intersect(match_quad', match_triple', 'rows', 'stable');
        
        % Procrustes with matching points between three and four view
        [~, new_quad, ~] = procrustes(triple(:, IB)', quad');
        new_quad = new_quad';
        
        % Save new quad view points
        updated_quad_models(order(i)) = {new_quad};
        
        
        % Reassign temporary working variables
        triple = triple_models{order(i+1),1};
        quad = new_quad;
        
        match_triple = triple_models{order(i+1), 2};
        match_quad = quad_models{order(i), 2}(2:4, :);
        
        % Find intersection of points between four and three view
        [~, ~, IB] = intersect(match_quad', match_triple', 'rows', 'stable');
        
        % Procrustes with matching points between three and four view
        [~, ~, trans] = procrustes(quad', triple(:, IB)');
        
        % Apply transform to entire three view model and save in cell array
        new_triple = trans.b * triple' * trans.T + trans.c(1, :);
        new_triple = new_triple';
        updated_triple_models(order(i+1)) = {new_triple};
        
        % Append transformed models to the complete model
        complete_model = [complete_model updated_triple_models{order(i+1)}];
%         complete_model = [complete_model updated_quad_models{order(i)} updated_triple_models{order(i+1)}];
    end
        
end





















