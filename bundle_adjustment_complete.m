% bundle_adjustment_complete.m
%
% Applies bundle adjustment to the complete 3D point model 
%
% Input:
%   -   model = complete 3D point model
%   -   pvm = point view matrix
%
% Output:
%   -   adj_model = adjusted 3D point model
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function bundle_adjustment_complete(pvm, keypoints)
    % Generate point track object for each point
    N = size(pvm, 2);
    point_tracks = {N, 1};
    for i = 1:N
        views = find(pvm(:, i));
        pts = zeros(length(views), 2);
        for j = length(views)
            pts(j, 1) = keypoints{views(j), 2}(pvm(views(j), i)); % save x coordinates of keypoint
            pts(j, 2) = keypoints{views(j), 3}(pvm(views(j), i)); % save y coordinates of keypoint
        end
        
        % save point tracks for every point
        point_tracks{i} = pointTrack(views, pts);
    end
    
    % determine camera parameters using the windows of the castle
    images =  imageSet('~/Documents/CV_Final/modelCastlePNG/');
    imageFileNames = images.ImageLocation;
    
    % detect calibration pattern
    [imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames);
    
    
    
    % apply bundle adjustment, model provided as columns vectors
%     adj_model = bundle_adjustment(model', );
end