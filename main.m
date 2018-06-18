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

close all; clear all; clc;


%% Tunable parameters
harris_scales       = 22; % determines how many scales the image is checked for
harris_threshold    = 0.00005;
nearest_neighbour   = 0.87;
sift_thresh         = 0.75;
ransac_iters        = 10000;
ransac_thresh       = 100;
 
% switches per step
step1               = 0; % Perform feature detection
step1_robot         = 0; % Perform feature detection with external feature detector
step1_vlsift        = 0; % Perform feature detection using vl_sift
step2               = 0; % Perform feature matching
step2_vlmatch       = 0; % Perform feature matching using vl_ubcmatch
step3               = 0; % Apply normalized 8-point RANSAC to find best matches
step3_matlab        = 1; % Apply normalized 8-point RANSAC to find best matches using MATLAB algorithm
step4               = 1; % Determine point view matrix
step5               = 1; % 3D coordinates for 3 and 4 consecutive images
step6               = 1; % Procrustes analysis
step7               = 0; % Bundle adjustment
step8               = 1; % Surface plot of complete model

% example plots
plots               = 0; % Show example plot of the keypoints found
image_plot          = 14; % Which images are plotted, this number indicates the left image


if(step1)
%% Step 1: create list of images, Detect feature points and create Sift descriptor
    fprintf('Perform feature point detection \n');

    % Create complete data cell
    % | name | x | y | s | d | matches | vl_Sift matches | RANSAC matches
    keypoints = {};
    folder_name = 'modelCastlePNG';

    % Create list of images
    keypoints(:,1) = loaddata(folder_name);
    fprintf("Files loaded into filenames \n")
    
    % Detect feature points and extract sift features
    num_im = length(keypoints);
    for i = 1:num_im
        fprintf(strcat("Starting image ",num2str(i)," of ",num2str(num_im)," \n"))
        [s,r,c] = extractfeatures(keypoints{i,1}, harris_scales, harris_threshold);
            
        % Create sift Descriptor
        [x,y,d] = sift_descriptor(keypoints{i,1},s,r,c);
        keypoints(i,2) = {x};
        keypoints(i,3) = {y};
        keypoints(i,4) = {s};
        keypoints(i,5) = {d};
        keypoints(i,6) = {impixel(imread(keypoints{i,1}),x,y)./255};
        
        % Print progression
        fprintf(strcat(num2str(length(x))+" keypoints found. \n"));
    end
    
    save keypoints keypoints
end


if(step1_robot)
%% Step 1: create list of images, Detect feature points and create Sift descriptor using robot script
    fprintf('Perform feature point detection using robot script \n');

    % create complete data cell
    % | name | x | y | s | d | matches | vl_Sift matches | RANSAC matches
    keypoints = {};
    folder_name = 'modelCastlePNG';

    % create list of images
    keypoints(:,1) = loaddata(folder_name);
    fprintf("Files loaded into filenames \n")
    
    % Detect feature points and extract sift features
    num_im = length(keypoints);
    for i = 1:num_im
        fprintf(strcat("Starting image ",num2str(i)," of ",num2str(num_im)," \n"))
        [x1 y1 a1 b1 c1 desc1 x2 y2 a2 b2 c2 desc2] = extract_features2(keypoints{i,1},0);
            
        %create sift Descriptor
        [x,y,d] = sift_descriptor(keypoints{i,1},s,r,c);
        x = [x1' x2'];
        y = [y1' y2'];
        keypoints(i,2) = {x};
        keypoints(i,3) = {y};
        keypoints(i,4) = {[a1' b1' c1']};
        keypoints(i,5) = {[desc1' desc2']};
        keypoints(i,6) = {impixel(imread(keypoints{i,1}),x,y)./255};
        fprintf(strcat(num2str(length([x1' x2']))+" keypoints found. \n"))
    end
    
    save keypoints keypoints
end


if(step1_vlsift)
%% Step 1: create list of images, Detect feature points and create Sift descriptor using vl_sift only
    fprintf('Perform featurepoint detection using vl_sift only \n');

    % Create complete data cell
    % | name | x | y | s | d | matches | vl_Sift matches | RANSAC matches
    keypoints = {};
    folder_name = 'modelCastlePNG';

    % Create list of images
    keypoints(:,1) = loaddata(folder_name);
    fprintf("Files loaded into filenames \n")
    
    % Detect feature points and extract sift features
    num_im = length(keypoints);
    for i = 1:num_im
        fprintf(strcat("Starting image ",num2str(i)," of ",num2str(num_im)," \n"))
        [f,d] = vl_sift(single(rgb2gray(imread(keypoints{i,1}))),'PeakThresh',sift_thresh);  
        keypoints(i,2) = {f(1,:)};
        keypoints(i,3) = {f(2,:)};
        keypoints(i,4) = {f(3,:)};
        keypoints(i,5) = {d};
        keypoints(i,6) = {impixel(imread(keypoints{i,1}),f(1,:),f(2,:))./255};
        fprintf(strcat(num2str(length(f(1,:)))+" keypoints found. \n"))
    end
    
    save keypoints keypoints
end


if(step2)
%% step 2: Look for matches in feature points    
    fprintf('Perform feature matching \n');
    
    load keypoints
    num_im = size(keypoints,1);
    matches   = {};
    
    % match each image with its consecutive image and write to data
    for i = 1:(num_im-1)
        fprintf(strcat("Started Matching on Image ", num2str(i)," \n"));
        match = match_features(keypoints{i,2},keypoints{i,3},keypoints{i,5},keypoints{i+1,2},keypoints{i+1,3},keypoints{i+1,5},nearest_neighbour);
        matches(i,1) = {match};
        fprintf(strcat(num2str(length(match(1,:)))+" matches found. \n"))
    end

    % perform match between last and first image and write to data
    fprintf(("Started matching on last image \n"));
    match = match_features(keypoints{num_im,2},keypoints{num_im,3},keypoints{num_im,5},keypoints{1,2},keypoints{1,3},keypoints{1,5},nearest_neighbour);
    matches(num_im,1) = {match};
    fprintf(strcat(num2str(length(match(1,:)))+" matches found. \n"))
    
    save own_matches matches
end


if(step2_vlmatch)
%% step 2: Look for matches in feature points using vl_ubcmatch
    fprintf('Perform feature matching using vl_ubcmatch \n');
    
    load keypoints
    num_im = size(keypoints,1);
    matches   = {};
    
    % match each image with its consecutive image and write to data
    for i = 1:(num_im-1)
        fprintf(strcat("Started Matching on Image ", num2str(i)," \n"))
        [match, scores]  = vl_ubcmatch(keypoints{i,5},keypoints{i+1,5},1/nearest_neighbour );
        matches(i,1) = {match};
        
        fprintf(strcat(num2str(length(match(1,:)))+" matches found. \n"))
    end
    
    % perform match between last and first image and write to data
    fprintf(("Started matching on last image \n"));
    [match, scores] = vl_ubcmatch(keypoints{num_im,5},keypoints{1,5},1/nearest_neighbour );
    matches(num_im,1) = {match};
    fprintf(strcat(num2str(length(match(1,:)))+" matches found. \n"))
    
    save vl_matches matches
end


if(step3)
%% Step 3: Apply 8-points RANSAC algorithm
    fprintf('Apply 8-point RANSAC \n');
    
    load keypoints
    num_im = size(keypoints,1);
    
    % Loop over all images except the last one
    for i = 1:(length(keypoints)-1)
        % normalize data
        x1 = keypoints{i,2}(matches{i,1}(1,:));
        y1 = keypoints{i,3}(matches{i,1}(1,:));
        x2 = keypoints{i+1,2}(matches{i,1}(2,:));
        y2 = keypoints{i+1,3}(matches{i,1}(2,:));
        [xn1,yn1,T1] = normalize_points(x1,y1);
        [xn2,yn2,T2] = normalize_points(x2,y2);
        
        % apply 8 point ransac algorithm
        [F, inliers] = fundamental_ransac(xn1,yn1,xn2,yn2,ransac_iters,ransac_thresh);
        FRD = T2' * F * T1; 
        fprintf(strcat(num2str(length(find(inliers)))+" inliers found. \n"))
    end
    
    % Process last and first image
    % normalize data
    x1 = keypoints{num_im,2}(matches{num_im,1}(1,:));
    y1 = keypoints{num_im,3}(matches{num_im,1}(1,:));
    x2 = keypoints{1,2}(matches{num_im,1}(2,:));
    y2 = keypoints{1,3}(matches{num_im,1}(2,:));
    [xn1,yn1,T1] = normalize_points(x1,y1);
    [xn2,yn2,T2] = normalize_points(x2,y2);
   
    % apply 8 point ransac algorithm
    [F, inliers] = fundamental_ransac(xn1,yn1,xn2,yn2,ransac_iters,ransac_thresh);
    FRD = T2' * F * T1; 
    fprintf(strcat(num2str(length(find(inliers)))+" inliers found. \n"))
    
    % save data
    save matches matches
end


if(step3_matlab)
%% Step 3: Apply 8-points RANSAC algorithm using MATLAB fundamental matrix algorithm
    fprintf('Apply 8-point RANSAC \n');
    
    load keypoints
    num_im = size(keypoints,1);
    
    % Loop over all images except the last one
    for i = 1:(length(keypoints)-1)
        % normalize data
        x1 = keypoints{i,2}(matches{i,1}(1,:));
        y1 = keypoints{i,3}(matches{i,1}(1,:));
        x2 = keypoints{i+1,2}(matches{i,1}(2,:));
        y2 = keypoints{i+1,3}(matches{i,1}(2,:));
        
        % apply 8 point ransac algorithm
        [FRD, inliers] = estimateFundamentalMatrix([x1',y1'],[x2',y2'],'method','RANSAC','NumTrials',ransac_iters,'DistanceThreshold',ransac_thresh);
        matches{i,2} = inliers';
        fprintf(strcat(num2str(length(find(inliers)))+" inliers found. \n"))
    end
    
    % Process last and first image
    % normalize data
    x1 = keypoints{num_im,2}(matches{num_im,1}(1,:));
    y1 = keypoints{num_im,3}(matches{num_im,1}(1,:));
    x2 = keypoints{1,2}(matches{num_im,1}(2,:));
    y2 = keypoints{1,3}(matches{num_im,1}(2,:));
   
    % apply 8 point ransac algorithm 
    [FRD, inliers] = estimateFundamentalMatrix([x1',y1'],[x2',y2'],'method','RANSAC','NumTrials',ransac_iters,'DistanceThreshold',ransac_thresh);
    matches{19, 2} = inliers';
    fprintf(strcat(num2str(length(find(inliers)))+" inliers found. \n"))
    
    % save data
    save matches matches
end


if(plots)
%% plot image for keypoint check
    fprintf('Plot keypoints on image \n');
    
    load keypoints
    load matches
    
    figure('name', stract('image_', sprintf('%d', image_plot)));

    imshow(imread(keypoints{image_plot ,1}));
    hold on
    x1 = keypoints{image_plot, 2};
    y1 = keypoints{image_plot, 3};
    color = keypoints{image_plot, 6};

    scatter(x1, y1, 10, color, '.')
end


if(step4)
%% Point view matrix
    fprintf('Find point view matrix \n');
    
    load keypoints
    load matches
    
    % point view matrix
    pvm = point_view_matrix(matches);
    
    save pvm pvm
end


if(step5)
%% 3D coordinates for 3 and 4 consecutive images
    fprintf('Perform affine structure from motion to determine 3D coordinates \n');

    load keypoints
    load matches
    load pvm

    skips = 0;
    % 3 consecutive images
    triple_im = [1:19; 2:19 1; 3:19 1 2];
    [triple_models,skips] = SfM(keypoints, PVM, triple_im,skips);
    skips
    
    % 4 consecutive images
    quad_im = [1:19; 2:19 1; 3:19 1 2; 4:19 1:3];
    [quad_models,skips] = SfM(keypoints, PVM, quad_im,skips);
    skips
    
    save triple_models triple_models
    save quad_models quad_models
end


if(step6)
%% Procrustes analysis for complete 3D model
    fprintf('Preform procrustes analysis to build complete 3D model \n');

    load own_triple_models
    load own_quad_models

    % Complete 3D model
    [complete_model, colors] = model_stitching(triple_models, quad_models);

    save complete_model complete_model
    save colors colors
end


if(step7)
%% Bundle Adjustment 
    fprintf('Perform bundle adjustment \n');

    load own_complete_model
    load own_pvm
    load own_keypoints
    load own_triple_models
    
    ba_model = bundle_adjustment_complete(PVM, keypoints, triple_models);
    
    save complete_model ba_model
end


if(step8)
%% Surface plot of complete model
    fprintf('Build surface plot of the complete model \n');

    load complete_model
    load colors
    
    % Plot 3D scatter plot of the complete model
    figure('name', 'Final model');
    scatter3(complete_model(1,:), complete_model(2,:), complete_model(3,:),'.b')
end























