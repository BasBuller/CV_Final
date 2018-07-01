%% SfM.m
%
% Generates the point view matrix based on feature point matches between
% several images.
%
% Input:
%   -   keypoints: keypoint coordinates for the respective images
%   -   pvm: point view matrix for the respective images
%   -   frames: order of the frames/images
%   -   skips: number of skips for non positive definite C matrices of
%               previous iterations
%
% Output:
%   -   models = 3D sub models
%   -   skips = total number of skips up until current iteration
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function [models, skips] = SfM(keypoints, pvm, frames, skips)
% Loop over all columns of the images matrix, such to cover all
% combinations of 3 or 4 consecutive images
models = cell(size(frames, 2), 6);

for i = 1:size(frames, 2)
    % Determine the point coordinates to be used during SfM, results in
    % using rows of PVM corresponding to required images
    match = pvm(frames(:, i), :);
    
    % Find columns of points that are not present in all consecutive images
    [~, col] = find(~match);
    
    % Point tracks
    track = pvm;
    track(:, unique(col)) = [];    
    models(i, 6) = {track};
    
    % Remove columns with zeros
    match(:, unique(col)) = [];
    
    pts = zeros(size(match, 1) * 2, size(match, 2));
    pts_n = zeros(size(match, 1) * 2, size(match, 2));
    
    % Fill pts with coordinates of keypoints
    for j = 1:size(match, 1)
        pts((2*j-1),:) = keypoints{frames(j, i), 2}(match(j, :)); 
        pts((2*j),:) = keypoints{frames(j, i), 3}(match(j, :));
        
    end
    
    % Remove mean
    for k = 1:size(pts,1)
        pts_n(k,:) = pts(k,:) - mean(pts(k,:));
    end 
    
    % make sure atleast 3 points are visible in all images
    if(size(pts, 2) > 2)
        
        %normalize points
        for k = 1:2:(size(pts_n,1)-1)
            x1 = pts_n(k,:) - mean(pts_n(k,:));
            y1 = pts_n(k+1,:) - mean(pts_n(k+1,:));
            pts_n(k,:) = x1;
            pts_n(k+1,:) = y1;
        end 
        
        % Determine SVD composition and reduce to rank 3
        [U, W, V]   = svd(pts_n);
        U3          = U(:, 1:3);
        W3          = W(1:3, 1:3); % Bugs out because there are no matchs covering 3 images
        V           = V';
        V3          = V(1:3, :);

        M           = U3 * sqrtm(W3);
        S           = sqrtm(W3) *  V3;

        save('M', 'M');
    
        % resolve affine ambiguity and solve for L
        A           = M(1:2, :);
        L0          = pinv(A'*A);
        options     = optimoptions(@lsqnonlin, 'StepTolerance',1e-16,'OptimalityTolerance',1e-16,'FunctionTolerance',1e-16);
        L           = lsqnonlin(@residuals, L0,[],[],options);%ones(size(L0))*-1e-2,ones(size(L0))*1e-2,options);
    
        if sum(real(eig(L))<0)>0
            fprintf(strcat("Eigenvalues to small size: ",num2str(size(match,2)),", round: ",num2str(i)," \n"))
            skips = skips +1;
            save('temp.mat','U3','W3','V','V3','M','S','A','L0','L','pts')
            
        else
        % Recover C
        C           = chol(L, 'lower');
       
        % Update M and S
        M           = M*C;
        S           = pinv(C)*S;
        end
        
        %Append to models cell array
        models(i,1) = {S};
        models(i,2) = {match};
        models(i,3) = {keypoints{frames(1,i),6}(match(1, :),:)};
        models(i,4) = {pts};            % coordinates of original keypoints
        models(i,5) = {M};
    end
end
end