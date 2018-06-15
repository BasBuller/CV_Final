% bundle_adjustment_complete.m
%
% Applies bundle adjustment to the complete 3D point model 
%
% Input:
%   -   model = complete 3D point model
%
% Output:
%   -   adj_model = adjusted 3D point model
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function bundle_adjustment_complete(model)
    
    
    % apply bundle adjustment, model provided as columns vectors
    adj_model = bundle_adjustment(model', );
end