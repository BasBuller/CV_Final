% SfM.m
%
% Generates the point view matrix based on feature point matches between
% several images.
%
% Input:
%   -   pts = dataset containing all centered points that are tracked, 
%             shape [m, n], m = number of views, n = number of points per view.
%
% Input:
%   -   keypoints = keypoint coordinates for the respective images
%   -   pvm = point view matrix for the respective images
%   -   n = amount of consecutive images used
%
% Output:
%   -   S = 3D Graph of the tracked points
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function S = SfM(keypoints, pvm)
% Determine the point coordinates to be used during SfM
pts = zeros(size(pvm, 1)*2, size(pvm, 2));
for i = 1:size(pvm, 1)
    pts(i, :) = keypoints(pvm(i));
    pts(i+1, :) = keypoints_y(pvm(i));
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
end