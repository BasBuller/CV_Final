% fundamental_matrix.m
%
% Generates the fundamental matrix based on matching features points
%
% Input:
%   - A[n, 9]: Contains the cross terms of the matching feature points
%
% Output:
%   - F[3, 3]: Fundamental matrix 
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function F = fundamental_matrix(A)
[~, ~, V] = svd(A);
F = V(:,end);               % Last column corresponds to column of smallest value in D, MATLAB puts singular values in decreasing order

F = reshape(F,[3,3])';      % Transpose because reshape inserts elements as columns instead of rows
[Uf,Df,Vf] = svd(F);
Df(3,3) = 0;                % Set smallest element to zero
     
F = Uf * Df * Vf';
end