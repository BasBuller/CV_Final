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

clear all;  


%% Tunable parameters
harris_scales       = 22; % determines how many scales the image is checked for
harris_threshold    = 0.00005;
nearest_neighbour   = 0.87;
sift_thresh         = 0.75;
ransac_iters        = 10000;
ransac_thresh       = 10;

own_algorithm       = 0; % Use sift feature detection and matching (0) or own algorithm (1)      
step1               = 0; % Perform feature detection
step2               = 0; % Perform feature matching
step3               = 0; % Apply normalized 8-point Ransac to find best matches
step4               = 0; % Determine point view matrix
step5               = 0; % 3D coordinates for 3 and 4 consecutive images
step6               = 1; % Procrustes analysis
step7               = 0; % Bundle adjustment
step8               = 0; % Surface plot of complete model
plots               = 0; % Show example plots
image1              = 1;% Which images are plotted, this number indicates the left image

if(step1)
%% Step 1: create list of images, Detect feature points and create Sift descriptor
    % create complete data cell
    % | name | x | y | s | d | matches | vl_Sift matches | RANSAC matches
    keypoints = {};
    folder_name = 'modelCastlePNG';

    % create list of images
    keypoints(:,1) = loaddata(folder_name);
    fprintf("Files loaded into filenames \n")
    
    % Detect feature points and extract sift features
    num_of_im = length(keypoints);
    for i = 1:num_of_im
        fprintf(strcat("Starting image ",num2str(i)," of ",num2str(num_of_im)," \n"))
%         if(own_algorithm)
            [s,r,c] = extractfeatures(keypoints{i,1},harris_scales,harris_threshold);
%             [x1 y1 a1 b1 c1 desc1 x2 y2 a2 b2 c2 desc2] = extract_features2(keypoints{i,1},0);
            
% %            create sift Descriptor
             [x,y,d] = sift_descriptor(keypoints{i,1},s,r,c);
            keypoints(i,2) = {x};
            keypoints(i,3) = {y};
            keypoints(i,4) = {s};
            keypoints(i,5) = {d};
            keypoints(i,6) = {impixel(imread(keypoints{i,1}),x,y)./255};
            fprintf(strcat(num2str(length(x))+" keypoints found. \n"))
%             x = [x1' x2'];
%             y = [y1' y2'];
%             keypoints(i,2) = {x};
%             keypoints(i,3) = {y};
%             keypoints(i,4) = {[a1' b1' c1']};
%             keypoints(i,5) = {[desc1' desc2']};
%             keypoints(i,6) = {impixel(imread(keypoints{i,1}),x,y)./255};
%             fprintf(strcat(num2str(length([x1' x2']))+" keypoints found. \n"))
%         else % using only vl_sift
%             [f,d] = vl_sift(single(rgb2gray(imread(keypoints{i,1}))),'PeakThresh',sift_thresh);  
%             keypoints(i,2) = {f(1,:)};
%             keypoints(i,3) = {f(2,:)};
%             keypoints(i,4) = {f(3,:)};
%             keypoints(i,5) = {d};
%         end
    end

%     if(own_algorithm)
%         save own_keypoints keypoints
%     else
        save vl_keypoints keypoints
%     end

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
        fprintf(strcat(num2str(length(match(1,:)))+" matches found. \n"))
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
     fprintf(strcat(num2str(length(match(1,:)))+" matches found. \n"))
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
        [FRD, inliers] = estimateFundamentalMatrix([x1',y1'],[x2',y2'],'method','RANSAC','NumTrials',ransac_iters,'DistanceThreshold',ransac_thresh);
        matches{i,2} = inliers';
        fprintf(strcat(num2str(length(find(inliers)))+" inliers found. \n"))
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
        [FRD, inliers] = estimateFundamentalMatrix([x1',y1'],[x2',y2'],'method','RANSAC','NumTrials',ransac_iters,'DistanceThreshold',ransac_thresh);
        matches{19, 2} = inliers';
        fprintf(strcat(num2str(length(find(inliers)))+" inliers found. \n"))
        % save data
    if(own_algorithm)
        save own_matches matches
    else
        save vl_matches matches
    end
   
%     % plotting results
% [lines] = epipolarLine(FRD, [x1(inliers)',y1(inliers)']);
% points = lineToBorderPoints(lines, size(imread(keypoints{2,1})));
% % 
% figure()
% imshow(imread(keypoints{1,1}))
% hold on
% scatter(x1(inliers),y1(inliers),'r')
% 
% figure()
% imshow(imread(keypoints{2,1}))
% hold on
% line(points(:,[1,3])',points(:,[2,4])');
%     
%     
%     
% end
end


if(plots)
%% plot image 
    if(own_algorithm)
        load own_keypoints
        load own_matches
    else
        load vl_keypoints
        load vl_matches
    end
    
%     if(own_algorithm)
%         figure('name','1 and 2 with own algorithm')
%     else
%         figure('name','1 and 2 with vl_sift')
%     end
%     imshow([imread(keypoints{image1,1}) imread(keypoints{image1+1,1})])
%     hold on
%     x1 = keypoints{image1,2}(matches{image1,1}(1,matches{image1,2}));
%     y1 = keypoints{image1,3}(matches{image1,1}(1,matches{image1,2}));
%     x2 = size(imread(keypoints{image1,1}),2)+keypoints{image1+1,2}(matches{image1,1}(2,matches{image1,2}));
%     y2 = keypoints{image1+1,3}(matches{image1,1}(2,matches{image1,2}));
% 
%     scatter(x1,y1,'r')
%     scatter(x2,y2,'r')
%     line([x1;x2],[y1;y2],'color','b')
%     size(x1)
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
    
    im1 = imread(keypoints{image1,1});
    im2 = imread(keypoints{image1+1,1});
    x1 = (keypoints{image1,2}(matches{image1,1}(1,matches{image1,2})));
    y1 = (keypoints{image1,3}(matches{image1,1}(1,matches{image1,2})));
    x2 = (keypoints{image1+1,2}(matches{image1,1}(2,matches{image1,2})));
    y2 = (keypoints{image1+1,3}(matches{image1,1}(2,matches{image1,2})));
    color = keypoints{image1,6}((matches{image1,1}(1,matches{image1,2})),:);
    figure()
%   imshow(imread(keypoints{image1,1})
    scatter(x1,-y1,10,color,'.')
    
    figure()

    
    showMatchedFeatures(im1, im2 ,[x1' y1'],[x2' y2'])

end


if(step4)
%% Point view matrix
    fprintf('Find point view matrix');
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
    skips = 0;
    % 3 consecutive images
    triple_im = [1:19; 2:19 1; 3:19 1 2];
    [triple_models,skips] = SfM(keypoints, PVM, triple_im,skips);
    skips
    % 4 consecutive images
    quad_im = [1:19; 2:19 1; 3:19 1 2; 4:19 1:3];
    [quad_models,skips] = SfM(keypoints, PVM, quad_im,skips);
    skips
    if(own_algorithm)
        save own_triple_models triple_models
        save own_quad_models quad_models
    else
        save vl_triple_models triple_models
        save vl_quad_models quad_models
    end
end


if(step6)
%% Procrustes analysis for complete 3D model
    if(own_algorithm)
        load own_triple_models
        load own_quad_models
    else
        load vl_triple_models
        load vl_quad_models
    end
    
    % Complete 3D model
    [quad_order, triple_order] = model_stitching(triple_models, quad_models);
    
    if(own_algorithm)
        save own_complete_model complete_model
        save colors colors
    else
        save vl_complete_model complete_model
        save colors colors
    end
end


if(step7)
%% Bundle Adjustment 
    if(own_algorithm)
        load own_complete_model
    else
        load vl_complete_model
    end
    
    if(own_algorithm)
        save own_complete_model complete_model
    else
        save vl_complete_model complete_model
    end
end


if(step8)
%% Surface plot of complete model
    if(own_algorithm)
        load own_complete_model
        load colors
    else
        load vl_complete_model
        load colors
    end
    
%     % Plot 3D scatter plot of the complete model
% figure()
% scatter3(complete_model(1,:), complete_model(2,:), complete_model(3,:),'.b')

figure()
scatter3(complete_model(1,:), complete_model(2,:), complete_model(3,:),[],colors,'.')

% [X,Y] = meshgrid(round(complete_model(1,:)), round(complete_model(2,:)));
x = complete_model(1,:);
y = complete_model(2,:);
z = complete_model(3,:);
rgb = complete_model();
% x = x - min(x);
% y = y - min(y);
% z = z -min(z);
% model = zeros(ceil(max(y)),ceil(max(x)));
% for i = 1: length(x)
%     model(round(y(i))+1,round(x(i))+1) = z(i);
%     
% end
% tri = delaunay(x,y);
% trisurf(tri,x,y,z)
% figure()
% surf(X,Y,model)
end



























