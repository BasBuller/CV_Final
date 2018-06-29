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
harris_threshold    = 0.001;
nearest_neighbour   = 0.80;
ransac_iters        = 30000;
ransac_thresh_own   = 0.0001;
ransac_thresh_mat   = 20;
dot_size            = 11;
max_iters_ba        = 50;
 
% switches per step
step1               = 0; % Perform feature detection
step1_robot         = 0; % Perform feature detection with external feature detector
step2               = 0; % Perform feature matching using vl_ubcmatch
step3               = 0; % Apply normalized 8-point RANSAC to find best matches
step4               = 0; % Determine point view matrix
step5               = 0; % 3D coordinates for 3 and 4 consecutive images
step6               = 1; % Perform local bundle adjustment
step7               = 1; % Procrustes analysis
step8               = 1; % Surface plot of complete model

% example plots
plots               = 0; % Show example plot of the keypoints found
image_plot          = 1; % Which images are plotted, this number indicates the left image
keypts              = 0;
eplns               = 1;


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
    folder_name = 'modelCastle_features';

    % create list of images
    keypoints(:,1) = loaddata(folder_name);
    fprintf("Files loaded into filenames \n")
    
    % Detect feature points and extract sift features
    num_im = length(keypoints);
    for i = 1:num_im
        fprintf(strcat("Starting image ",num2str(i)," of ",num2str(num_im)," \n"))
        [x1 y1 a1 b1 c1 desc1 x2 y2 a2 b2 c2 desc2] = extract_features2(keypoints{i,1},0);
            
        %create sift Descriptor
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


if(step2)
%% step 2: Look for matches in feature points using vl_ubcmatch
    fprintf('Perform feature matching using vl_ubcmatch \n');
   
    load keypoints
    num_im = size(keypoints,1);
    matches   = {};
    
    % match each image with its consecutive image and write to data
    for i = 1:(num_im-1)
        fprintf(strcat("Started Matching on Image ", num2str(i)," \n"))
        [match, ~]  = vl_ubcmatch(keypoints{i,5},keypoints{i+1,5},1/nearest_neighbour );
        matches(i,1) = {match};
        
        fprintf(strcat(num2str(length(match(1,:)))+" matches found. \n"))
    end
    
    % perform match between last and first image and write to data
    fprintf(("Started matching on last image \n"));
    [match, ~] = vl_ubcmatch(keypoints{num_im,5},keypoints{1,5},1/nearest_neighbour );
    matches(num_im,1) = {match};
    fprintf(strcat(num2str(length(match(1,:)))+" matches found. \n"))
    
    save matches matches
end


if(step3)
%% Step 3: Apply 8-points RANSAC algorithm
    fprintf('Apply 8-point RANSAC \n');
    
    load keypoints
    load matches
    num_im = size(keypoints,1);
    FM = cell(num_im, 1);
    
    % Loop over all images except the last one
    for i = 1:(length(keypoints)-1)
        fprintf(strcat("Starting on image: ", sprintf("%d", i), "\n"));
        
        % normalize data
        x1 = keypoints{i,2}(matches{i,1}(1,:));
        y1 = keypoints{i,3}(matches{i,1}(1,:));
        x2 = keypoints{i+1,2}(matches{i,1}(2,:));
        y2 = keypoints{i+1,3}(matches{i,1}(2,:));
        [xn1,yn1,T1] = normalize_points(x1,y1);
        [xn2,yn2,T2] = normalize_points(x2,y2);
        
        % apply 8 point ransac algorithm
        [F, inliers] = fundamental_ransac(xn1,yn1,xn2,yn2,ransac_iters,ransac_thresh_own);
        FRD = T2' * F * T1; 
        matches(i, 2) = {inliers};
        FM(i, 1) = {FRD};
        fprintf(strcat(num2str(length(find(inliers)))+" inliers found. \n"))
    end
    
    % Process last and first image
    fprintf(strcat("Starting on image: ", sprintf("%d", num_im), "\n"));
    
    % normalize data
    x1 = keypoints{num_im,2}(matches{num_im,1}(1,:));
    y1 = keypoints{num_im,3}(matches{num_im,1}(1,:));
    x2 = keypoints{1,2}(matches{num_im,1}(2,:));
    y2 = keypoints{1,3}(matches{num_im,1}(2,:));
    [xn1,yn1,T1] = normalize_points(x1,y1);
    [xn2,yn2,T2] = normalize_points(x2,y2);
   
    % apply 8 point ransac algorithm
    [F, inliers] = fundamental_ransac(xn1,yn1,xn2,yn2,ransac_iters,ransac_thresh_own);
    FRD = T2' * F * T1; 
    matches(num_im, 2) = {inliers};
    FM(num_im, 1) = {FRD};
    fprintf(strcat(num2str(length(find(inliers)))+" inliers found. \n"))
    
    % save data
    save matches matches
    save FM FM
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
    [triple_models, skips] = SfM(keypoints, pvm, triple_im, skips);
    skips
    
    % 4 consecutive images
    quad_im = [1:19; 2:19 1; 3:19 1 2; 4:19 1:3];
    [quad_models, skips] = SfM(keypoints, pvm, quad_im, skips);
    skips
    
    save triple_models triple_models
    save quad_models quad_models
end


if(step6)
%% Local bundle adjustment 
    fprintf('Perform local bundle adjustment \n');
    
    load triple_models
    load quad_models
    
    % Triple view models
    for i = 1:max(size(triple_models))
        if (triple_models{i, 5})
            % Initialize parameters
            S = triple_models{i, 1};
            M_loc = triple_models{i, 5}';
            X0 = [M_loc S];
            key_pts = triple_models{i, 4};

            % Perform local BA
            options = optimoptions(@fminunc, 'Display', 'iter');
            MS = fminunc(@(x)ba_local(x, key_pts, 3), X0, options);
            M_loc = MS(:, 1:6)';
            S = MS(:, 7:end);

            % Update models
            triple_models(i, 1) = {S};
            triple_models(i, 5) = {M_loc};
        end
    end
    
    % Quad view models
    for i = 1:max(size(quad_models))
        if (quad_models{i, 5})
            % Initialize parameters
            S = quad_models{i, 1};
            M_loc = quad_models{i, 5}';
            X0 = [M_loc S];
            key_pts = quad_models{i, 4};

            % Perform local BA
            options = optimoptions(@fminunc, 'Display', 'iter');
            MS = fminunc(@(x)ba_local(x, key_pts, 4), X0, options);
            M_loc = MS(:, 1:8)';
            S = MS(:, 9:end);

            % Update models
            quad_models(i, 1) = {S};
            quad_models(i, 5) = {M_loc};
        end
    end
    
    save triple_models triple_models
    save quad_models quad_models
end


if(step7)
%% Procrustes analysis for complete 3D model
    fprintf('Preform procrustes analysis to build complete 3D model \n');

    load triple_models
    load quad_models

    % Complete 3D model
    [complete_model, colors, quad_order, triple_order, updated_triple_models, updated_quad_models] = model_stitching(triple_models, quad_models);

    save complete_model complete_model
    save colors colors
    save triple_order triple_order
    save quad_order quad_order
    save updated_triple_models updated_triple_models
    save updated_quad_models updated_quad_models
end
    

if(step8)
%% Surface plot of complete model
    fprintf('Build surface plot of the complete model without ba \n');

    load complete_model
    load colors
    
    % Plot 3D scatter plot of the complete model
    x = complete_model(1,:);
    y = complete_model(2,:);
    z = complete_model(3,:);
    colors = uint8(colors.*255);
    castle = pointCloud([x' y' z']);
    castle.Color = colors;

    denoised = pcdenoise(castle);
    figure('Name','Original')
    pcshow(castle, 'MarkerSize', dot_size)
    figure('Name','Denoised')
    pcshow(denoised, 'MarkerSize', dot_size)
end


if(plots)
%% plot image for keypoint check
    fprintf('Plot keypoints on image \n');
    
    load keypoints
    load matches
    load FM
    
    figure('name', strcat('image_', sprintf('%d', image_plot)));

    imgs = [1:19 1];
    Im1 = imread(keypoints{imgs(image_plot), 1});
    Im2 = imread(keypoints{imgs(image_plot+1), 1});
    imshow(Im2);
    hold on
    
    % Plot keypoints
    if(keypts)
        x1 = keypoints{image_plot, 2};
        y1 = keypoints{image_plot, 3};
        color = keypoints{image_plot, 6};
        scatter(x1, y1, 10, color, '.')
    end
    
    % Plot epipolar lines
    if(eplns)
        epilines = epipolarLine(FM{image_plot, 1}, [keypoints{image_plot, 2}(1, matches{image_plot, 2})' keypoints{image_plot, 3}(1, matches{image_plot, 2})']);
        points = lineToBorderPoints(epilines, size(Im2));
        line(points(:,[1,3])',points(:,[2,4])');
    end
end






















