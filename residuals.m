% residuals.m
%
% Computes the residuals for every camera point of view.
%
% Input:
%   -   L = the matrix L that will be minimized for Ai*L*Ai' = eye(2)
%   -   M = movement matrix of the svd decomposition
%
% Output:
%   -   dif= a matrix (n x 4) containing the residuals for the n cameras
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function dif = residuals(L)

%load the saved transformation matrix M
load('M', 'M');

%pre-alocate the dif matrix
dif = zeros(size(M,1)/2,4);

%compute the residuals
for i = 1:size(M,1)/2
    Ai = M(i*2-1:i*2,:);
    D = (Ai*L*Ai' - eye(2));
    dif(i,:) = D(:);
    
end