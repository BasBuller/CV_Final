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

function models = SfM(keypoints, pvm, images)
% Loop over all columns of the images matrix, such to cover all
% combinations of 3 or 4 consecutive images
models = {};

for i = 1:size(images, 2)
    % Determine the point coordinates to be used during SfM
    match = pvm(images(:, i), :);
    pts = zeros(size(match,1) * 2, size(match,2));
    
    % Fill pts with coordinates of keypoints
    for j = 1:size(images, 1)
        pts((2*j-1),:) = keypoints{images(j, i),2}(j, match(j, :)); % indexing goes wrong here, matlab cannot take 0 as index
        pts((2*j),:) = keypoints{images(j,i),3}(j, match(j, :));
    end

    % Determine SVD composition and reduce to rank 3
    [U, W, V]   = svd(pts);
    U3          = U(:, 1:3);
    W3          = W(1:3, 1:3);
    V           = V';
    V3          = V(1:3, :);

    M           = U3 * sqrt(W3);
    S           = sqrt(W3) *  V3;

    % resolve affine ambiguity and solve for L
    L0          = pinv(A'*A);
    L           = lsqnonlin(@myfun, L0);

    % Recover C
    C           = chol(L, 'lower');

    % Update M and S
    M           = M*C;
    S           = pinv(C)*S;
    
    % Append to models cell array
    models(i,1) = {S};
end

end