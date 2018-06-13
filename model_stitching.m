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
        if(size(triple_models{i}, 2) > len)
            len = size(triple_models{i,1}, 2);
            ind = i;
        end
    end
    
    % Set initial model
    
end