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
%   -   model = complete 3D model
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function model = model_stitching(triple_models, quad_models)
    % Find biggest model, set as starting point
    ind = 0;
    len = 0;
    for i = 1:size(triple_models, 2)
        if(size(triple_models{i,1}, 2) > len)
            len = size(triple_models{i,1}, 2);
            ind = i;
        end
    end
    
    % Set initial model
    model = triple_models{ind, 1};
    
    % Loop over the models based on three images
    for i = 2:size(triple_models, 1)
        if (size(triple_models{i,1}, 2) > 0)
            % New points
            new = triple_models{i,1};
            [~, Z, ~] = procrustes(model, new);
            
            % Append to existing model
            model = [model Z];
        end
    end
    
    % Loop over the models based on four images
    for i = 1:size(quad_models, 1)
        if (size(quad_models{i,1}, 2) > 0)
            % New points
            new = quad_models{i,1};
            [~, Z, ~] = procrustes(model, new);
            
            % Append to existing model
            model = [model Z];
        end
    end
end