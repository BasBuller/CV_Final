%% Load images for final assignment
% create list of images present for the structure from motion project
% Input: 
%   -foldername
%
% Output: 
%   -3D model of the object
%
% Authors: 
%   -Bas Buller 4166566
%   -Rick Feith 4218272

clear all
close all
%% Tunable parameters
harris_scales       = 7; % determines how many scales the image is checked for
harris_threshold    = 0.0005;
nearest_neighbour   = 0.8;
sift_thresh         = 1.5;
ransac_iters        = 2000;
ransac_thresh       = 1e-02;

own_algorithm       = 0; % use sift feature detection and matching (0) or own algorithm (1)      
step1               = 0; % Perform feature detection
step2               = 0; % Perform feature matching
step3               = 0; % Apply normalized 8-point Ransac to find best matches
plots               = 0; % show example plots

tic
%% create complete data cell
if(step1)
    % | name | x | y | s | d | matches | vl_Sift matches | RANSAC matches
    
    data = {};

    %% Step 1: create list of images, Detect feature points and create Sift descriptor
    folder_name = 'model_castle';

    % create list of images
    data(:,1) = loaddata(folder_name);
    fprintf("Files loaded into filenames \n")
    %% Detect feature points and extract sift features
    num_of_im = length(data);
    for i = 1:num_of_im
        fprintf(strcat("Starting image ",num2str(i)," of ",num2str(num_of_im)," \n"))
        if(own_algorithm)
        [s,r,c] = extractfeatures(data{i,1},harris_scales,harris_threshold);
        % create sift Descriptor
        [x,y,d] = sift_descriptor(data{i,1},s,r,c);
        data(i,2) = {x};
        data(i,3) = {y};
        data(i,4) = {s};
        data(i,5) = {d};
        else % using only vl_sift
        [f,d] = vl_sift(single(rgb2gray(imread(data{i,1}))),'PeakThresh',sift_thresh);  
        data(i,2) = {f(1,:)};
        data(i,3) = {f(2,:)};
        data(i,4) = {f(3,:)};
        data(i,5) = {d};
        end
    end

if(own_algorithm)
    save own_data data
else
    save vl_data data
end
end

%% step 2: Look for matches in feature points
if(step2)
    if(own_algorithm)
    load own_data
    else
    load vl_data
    end
    num_of_im = size(data,1);
    % match each image with its consecutive image and write to data
    for i = 1:(num_of_im-1)
        fprintf(strcat("Started Matching on Image ", num2str(i)," \n"));
        if(own_algorithm)
        matches = match_features(data{i,2},data{i,3},data{i,5},data{i+1,2},data{i+1,3},data{i+1,5},nearest_neighbour);
        data(i,6) = {matches};
        else
        [match scores]  = vl_ubcmatch(data{i,5},data{i+1,5},1/nearest_neighbour );
        data(i,6) = {match};
        end
    end
    % perform match between last and first image and write to data
    fprintf(("Started Matching on Image 19 \n"));
    if(own_algorithm)
    matches = match_features(data{19,2},data{19,3},data{19,5},data{1,2},data{1,3},data{1,5},nearest_neighbour);
    data(19,6) = {matches};
    save own_data data
    else
    [match scores] = vl_ubcmatch(data{19,5},data{1,5},1/nearest_neighbour );
    data(19,6) = {match};
    save vl_data data
    end
    


end

%% Step 3: Apply 8-points RANSAC algorithm

%load data
if(step3)
    if(own_algorithm)
    load own_data
    else
    load vl_data
    end
    
    for i = 1:(length(data)-1)
    % normalize data
        x1 = data{i,2}(data{i,6}(1,:));
        y1 = data{i,3}(data{i,6}(1,:));
        x2 = data{i+1,2}(data{i,6}(2,:));
        y2 = data{i+1,3}(data{i,6}(2,:));
        [xn1,yn1,T1] = normalize_points(x1,y1);
        [xn2,yn2,T2] = normalize_points(x2,y2);
      
      % apply 8 point ransac algorithm
      % [F, inliers] = fundamental_ransac(xn1,yn1,xn2,yn2,ransac_iters,ransac_thresh);
%       FRD = T2' * F * T1; 
        [FRD, inliers] = estimateFundamentalMatrix([x1',y1'],[x2',y2'],'method','RANSAC','NumTrials',2000,'DistanceThreshold',ransac_thresh);
        data{i,7} = inliers;
    end
    
        % normalize data
        x1 = data{19,2}(data{19,6}(1,:));
        y1 = data{19,3}(data{19,6}(1,:));
        x2 = data{1,2}(data{19,6}(2,:));
        y2 = data{1,3}(data{19,6}(2,:));
        [xn1,yn1,T1] = normalize_points(x1,y1);
        [xn2,yn2,T2] = normalize_points(x2,y2);
      
      % apply 8 point ransac algorithm
      % [F, inliers] = fundamental_ransac(xn1,yn1,xn2,yn2,ransac_iters,ransac_thresh);
%       FRD = T2' * F * T1; 
        [FRD, inliers] = estimateFundamentalMatrix([x1',y1'],[x2',y2'],'method','RANSAC','NumTrials',2000,'DistanceThreshold',ransac_thresh);
        data{19,7} = inliers;
  
        % save data
    if(own_algorithm)
        save own_data data
    else
        save vl_data data
    end
end    
%     % plotting results
% [lines] = epipolarLine(FRD, [x1(inliers)',y1(inliers)']);
% points = lineToBorderPoints(lines, size(imread(data{2,1})));
% 
% figure()
% imshow(imread(data{1,1}))
% hold on
% scatter(x1(inliers),y1(inliers),'r')
% 
% figure()
% imshow(imread(data{2,1}))
% hold on
% line(points(:,[1,3])',points(:,[2,4])');
%     
%     
%     
% end

toc


%% plot image for check using first 20 matches
if(plots)
    if(own_algorithm)
    load own_data
    else
    load vl_data
    end
    
    if(own_algorithm)
    figure('name','1 and 2 with own algorithm')
    else
    figure('name','1 and 2 with vl_sift')
    end
imshow([imread(data{1,1}) imread(data{2,1})])
hold on
x1 = data{1,2}(data{1,6}(1,1:100));
y1 = data{1,3}(data{1,6}(1,1:100));
x2 = size(imread(data{1,1}),2)+data{2,2}(data{1,6}(2,1:100));
y2 = data{2,3}(data{1,6}(2,1:100));

scatter(x1,y1,'r')
scatter(x2,y2,'r')
line([x1;x2],[y1;y2],'color','b')

% figure('name','1 and 2 with vl_sift algorithm')
% imshow([imread(data{1,1}) imread(data{2,1})])
% hold on
% x1 = data{1,2}(data{1,7}(1,1:20));
% y1 = data{1,3}(data{1,7}(1,1:20));
% x2 = size(imread(data{1,1}),2)+data{2,2}(data{1,7}(2,1:20));
% y2 = data{2,3}(data{1,7}(2,1:20));
% 
% scatter(x1,y1,'r')
% scatter(x2,y2,'r')
% line([x1;x2],[y1;y2],'color','b')


end