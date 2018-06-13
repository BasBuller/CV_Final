% fundamental_ransac.m
%
% Applies RANSAC to the fundamental matrix
%
% Input:
%   - x1[n, 1]: x coordinates of feature points in the initial image
%   - y1[n, 1]: y coordinates of feature points in the initial image
%   - x2[n, 1]: x coordinates of feature points in the matching image
%   - y2[n, 1]: y coordinates of feature points in the matching image
%   - N: number of RANSAC iterations
%   - threshold: inlier threshold for RANSAC
%
% Output:
%   - F[3, 3]: Fundamental matrix after RANSAC
%   - inliers: array containing the indices of the inliers within the set
%     of matching feature points.
%
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function [F, inliers] = fundamental_ransac(x1,y1,x2,y2,N,threshold)
%% Initialize parameters
inliers         = [];
total_inliers   = 0;
P               = 8;

%% Loop to find optimal Fundamental Matrix
for run = 1:N
    seed = randperm(max(size(x1)), P);      % select P random feature point matches
       
    A  = zeros(P,9);
    for i = seed
        A(i,:) = [x2(i)*x1(i) x2(i)*y1(i) x2(i) y2(i)*x1(i) y2(i)*y1(i) y2(i) x1(i) y1(i) 1]; 
    end
    Fi = fundamental_matrix(A);              % Initial fundamental matrix
        
    p1 = [x1;y1;ones(1,length(x1))];
    p2 = [x2;y2;ones(1,length(x2))];
    Fp1 = Fi*p1;
    FTp1 = Fi'*p1;
        
    num = diag(p2'*Fi*p1).^2;                                                     % Sampson distance numerator
    den = (Fp1(1,:)).^2 + (Fp1(2,:)).^2 + (FTp1(1,:)).^2 + (FTp1(2,:)).^2;          % Sampson distance denominator
    inl     = find(num'./den < threshold);
        
    if length(inl) > total_inliers
        total_inliers    = length(inl);
        inliers         = inl;
    end    
end

A  = zeros(total_inliers,9);
for i = 1:total_inliers
    A(i,:) = [x2(inliers(i))*x1(inliers(i)) x2(inliers(i))*y1(inliers(i)) x2(inliers(i)) y2(inliers(i))*x1(inliers(i)) y2(inliers(i))*y1(inliers(i)) y2(inliers(i)) x1(inliers(i)) y1(inliers(i)) 1]; 
end
F  = fundamental_matrix(A);

end