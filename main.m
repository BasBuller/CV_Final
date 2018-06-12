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

clear all; close all;


%% Tunable parameters
harris_scales       = 7; % determines how many scales the image is checked for
harris_threshold    = 0.0005;
nearest_neighbour   = 0.8;
sift_thresh         = 1;
ransac_iters        = 2000;
ransac_thresh       = 1e-01;

own_algorithm       = 0; % Use sift feature detection and matching (0) or own algorithm (1)      
step1               = 0; % Perform feature detection
step2               = 0; % Perform feature matching
step3               = 0; % Apply normalized 8-point Ransac to find best matches
step4               = 0; % Determine point view matrix
step5               = 1; % 3D coordinates for 3 and 4 consecutive images
plots               = 0; % Show example plots


if(step1)
%% Step 1: create list of images, Detect feature points and create Sift descriptor
    % create complete data cell
    % | name | x | y | s | d | matches | vl_Sift matches | RANSAC matches
    keypoints = {};
    folder_name = 'model_castle';

    % create list of images
    keypoints(:,1) = loaddata(folder_name);
    fprintf("Files loaded into filenames \n")
    
    % Detect feature points and extract sift features
    num_of_im = length(keypoints);
    for i = 1:num_of_im
        fprintf(strcat("Starting image ",num2str(i)," of ",num2str(num_of_im)," \n"))
        if(own_algorithm)
            [s,r,c] = extractfeatures(keypoints{i,1},harris_scales,harris_threshold);
            
            % create sift Descriptor
            [x,y,d] = sift_descriptor(keypoints{i,1},s,r,c);
            keypoints(i,2) = {x};
            keypoints(i,3) = {y};
            keypoints(i,4) = {s};
            keypoints(i,5) = {d};
            
        else % using only vl_sift
            [f,d] = vl_sift(single(rgb2gray(imread(keypoints{i,1}))),'PeakThresh',sift_thresh);  
            keypoints(i,2) = {f(1,:)};
            keypoints(i,3) = {f(2,:)};
            keypoints(i,4) = {f(3,:)};
            keypoints(i,5) = {d};
        end
    end

    if(own_algorithm)
        save own_keypoints keypoints
    else
        save vl_keypoints keypoints
    end
end


if(step2)
%% step 2: Look for matches in feature points    
    if(own_algorithm)
        load own_keypoints
    else
        load vl_keypoints
    end
    num_of_im = size(keypoints,1);
    matches   = {};
    
    % match each image with its consecutive image and write to data
    for i = 1:(num_of_im-1)
        fprintf(strcat("Started Matching on Image ", num2str(i)," \n"));
        if(own_algorithm)
            match = match_features(keypoints{i,2},keypoints{i,3},keypoints{i,5},keypoints{i+1,2},keypoints{i+1,3},keypoints{i+1,5},nearest_neighbour);
            matches(i,1) = {match};
        else
            [match, scores]  = vl_ubcmatch(keypoints{i,5},keypoints{i+1,5},1/nearest_neighbour );
            matches(i,1) = {match};
        end
    end

    % perform match between last and first image and write to data
    fprintf(("Started Matching on Image 19 \n"));
     if(own_algorithm)
        match = match_features(keypoints{19,2},keypoints{19,3},keypoints{19,5},keypoints{1,2},keypoints{1,3},keypoints{1,5},nearest_neighbour);
        matches(19,1) = {match};
        save own_matches matches
    else
        [match, scores] = vl_ubcmatch(keypoints{19,5},keypoints{1,5},1/nearest_neighbour );
        matches(19,1) = {match};
        save vl_matches matches
     end
end


if(step3)
%% Step 3: Apply 8-points RANSAC algorithm

%load data
    if(own_algorithm)
        load own_keypoints
        load own_matches
    else
        load vl_keypoints
        load vl_matches
    end
    
    for i = 1:(length(keypoints)-1)
    % normalize data
        x1 = keypoints{i,2}(matches{i,1}(1,:));
        y1 = keypoints{i,3}(matches{i,1}(1,:));
        x2 = keypoints{i+1,2}(matches{i,1}(2,:));
        y2 = keypoints{i+1,3}(matches{i,1}(2,:));
        [xn1,yn1,T1] = normalize_points(x1,y1);
        [xn2,yn2,T2] = normalize_points(x2,y2);
      
      % apply 8 point ransac algorithm
      % [F, inliers] = fundamental_ransac(xn1,yn1,xn2,yn2,ransac_iters,ransac_thresh);
%       FRD = T2' * F * T1; 
        [FRD, inliers] = estimateFundamentalMatrix([x1',y1'],[x2',y2'],'method','RANSAC','NumTrials',2000,'DistanceThreshold',ransac_thresh);
        matches{i,2} = inliers';
    end
    
        % normalize data
        x1 = keypoints{19,2}(matches{19,1}(1,:));
        y1 = keypoints{19,3}(matches{19,1}(1,:));
        x2 = keypoints{1,2}(matches{19,1}(2,:));
        y2 = keypoints{1,3}(matches{19,1}(2,:));
        [xn1,yn1,T1] = normalize_points(x1,y1);
        [xn2,yn2,T2] = normalize_points(x2,y2);
      
      % apply 8 point ransac algorithm
      % [F, inliers] = fundamental_ransac(xn1,yn1,xn2,yn2,ransac_iters,ransac_thresh);
%       FRD = T2' * F * T1; 
        [FRD, inliers] = estimateFundamentalMatrix([x1',y1'],[x2',y2'],'method','RANSAC','NumTrials',2000,'DistanceThreshold',ransac_thresh);
        matches{19, 2} = inliers';
  
        % save data
    if(own_algorithm)
        save own_matches matches
    else
        save vl_matches matches
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
end


if(plots)
%% plot image for check using first 20 matches
    if(own_algorithm)
        load own_keypoints
        load own_matches
    else
        load vl_keypoints
        load vl_matches
    end
    
    if(own_algorithm)
        figure('name','1 and 2 with own algorithm')
    else
        figure('name','1 and 2 with vl_sift')
    end
    imshow([imread(keypoints{1,1}) imread(keypoints{2,1})])
    hold on
    x1 = keypoints{1,2}(matches{1,1}(1,matches{1,2}));
    y1 = keypoints{1,3}(matches{1,1}(1,matches{1,2}));
    x2 = size(imread(keypoints{1,1}),2)+keypoints{2,2}(matches{1,1}(2,matches{1,2}));
    y2 = keypoints{2,3}(matches{1,1}(2,matches{1,2}));

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


if(step4)
%% Point view matrix
    if(own_algorithm)
        load own_keypoints
        load own_matches
    else
        load vl_keypoints
        load vl_matches
    end
    
    % point view matrix
    PVM = point_view_matrix(matches);
    
    if(own_algorithm)
        save own_pvm PVM
    else
        save vl_pvm PVM
    end
end


if(step5)
%% 3D coordinates for 3 and 4 consecutive images
    if(own_algorithm)
        load own_keypoints
        load own_matches
        load own_pvm
    else
        load vl_keypoints
        load vl_matches
        load vl_pvm
    end
    
    % 3 consecutive images
    three_im = [1:19; 2:19 1; 3:19 1 2];
    three_models = SfM(keypoints, PVM, three_im);
    
    % 4 consecutive images
    quad_im = [1:19; 2:19 1; 3:19 1 2; 4:19 1:3];
    quad_models = SfM(keypoints, PVM, quad_im);
    
    if(own_algorithm)
        save own_triple_matches triple_matches
        save own_quad_matches quad_matches
    else
        save vl_triple_matches triple_matches
        save vl_quad_matches quad_matches
    end
end






























