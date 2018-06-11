% SfM.m
%
% Generates the point view matrix based on feature point matches between
% several images.
%
% Input:
%   -   pts = dataset containing all centered points that are tracked, 
%             shape [m, n], m = number of views, n = number of points per view.
%
% Output:
%   -   S = 3D Graph of the tracked points
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function S = SfM(pts)
% Determine SVD composition and reduce to rank 3
[U, W, V]   = svd(pts);
U3          = U(:, 1:3);
W3          = W(1:3, 1:3);
V           = V';
V3          = V(1:3, :);
    
M           = U3 * sqrt(W3);
S           = sqrt(W3) *  V3;

save('M', 'M');

% resolve affine ambiguity and solve for L
L0          = pinv(A'*A);
L           = lsqnonlin(@myfun, L0);

% Recover C
C           = chol(L, 'lower');

% Update M and S
M           = M*C;
S           = pinv(C)*S;
end