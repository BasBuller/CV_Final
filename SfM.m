% SfM.m
%
% Generates the point view matrix based on feature point matches between
% several images.
%
% Input:
%   -   keypoints = keypoint coordinates for the respective images
%   -   pvm = point view matrix for the respective images
%   -   images = consecutive images used
%
% Output:
%   -   S = 3D Graph of the tracked points
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function [models,skips] = SfM(keypoints, pvm, frames,skips)
% Loop over all columns of the images matrix, such to cover all
% combinations of 3 or 4 consecutive images
models = cell(size(frames, 2), 2);

for i = 1:size(frames, 2)
    % Determine the point coordinates to be used during SfM
    match = pvm(frames(:, i), :);
    
    % Find columns of points that are not present in all consecutive images
    [~, col] = find(~match);
    
    % Remove columns with zeros
    match(:, unique(col)) = [];
    
    pts = zeros(size(match,1) * 2, size(match,2));
    % Fill pts with coordinates of keypoints
    for j = 1:size(match, 1)
        pts((2*j-1),:) = keypoints{frames(j, i),2}(match(j, :)); 
        pts((2*j),:) = keypoints{frames(j,i),3}(match(j, :));
    end
    
    %normalize points
    for k = 1:size(pts,1)
        pts(k,:) = pts(k,:) - mean(pts(k,1));
    end 
    % make sure atleast 3 points are visible in all images
    if(size(pts, 2) > 2)    
        % Determine SVD composition and reduce to rank 3
        [U, W, V]   = svd(pts);
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
        options = optimoptions(@lsqnonlin, 'StepTolerance',1e-16,'OptimalityTolerance',1e-16,'FunctionTolerance',1e-16);
        L           = lsqnonlin(@residuals, L0,ones(size(L0))*-1e-2,ones(size(L0))*1e-2,options);
    
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
        
    end
end

end