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


%% Tunable parameters
harris_scales       = 7; % determines how many scales the image is checked for
harris_threshold    = 0.0005;
nearest_neighbour   = 0.8;

step1               = 1;
step2               = 1;
plots               = 0;
tic
%% create complete data cell
if(step1)
    % | name | x | y | s | d | matches | vl_Sift matches
    
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
        [s,r,c] = extractfeatures(data{i,1},harris_scales,harris_threshold);
        % create sift Descriptor
        [x,y,d] = sift_descriptor(data{i,1},s,r,c);
%         [f,d] = vl_sift(single(rgb2gray(imread(data{i,1}))));
        data(i,2) = {x};
        data(i,3) = {y};
        data(i,4) = {s};
        data(i,5) = {d};
%         data(i,2) = {f(1,:)};
%         data(i,3) = {f(2,:)};
%         data(i,4) = {f(3,:)};
%         data(i,5) = {d};
end

save data data
end

%% step 2: Look for matches in feature points
if(step2)
    load data
    num_of_im = size(data,1);
    % match each image with its consecutive image and write to data
    for i = 1:(num_of_im-1)
        fprintf(strcat("Started Matching on Image ", num2str(i)," \n"));
        matches = match_features(data{i,2},data{i,3},data{i,5},data{i+1,2},data{i+1,3},data{i+1,5},nearest_neighbour);
        data(i,6) = {matches};
%         [match scores]  = vl_ubcmatch(data{i,5},data{i+1,5});
%         data(i,6) = {match};
    end
    % perform match between last and first image and write to data
    fprintf(("Started Matching on Image 19 \n"));
    matches = match_features(data{19,2},data{19,3},data{19,5},data{1,2},data{1,3},data{1,5},nearest_neighbour);
    data(19,6) = {matches};
%     [match scores] = vl_ubcmatch(data{19,5},data{1,5});
%     data(19,6) = {match};
    
end

save data data
toc
%% plot image for check
if(plots)
figure('name','1 and 2 with own algorithm')
imshow([imread(data{1,1}) imread(data{2,1})])
hold on
x1 = data{1,2}(data{1,6}(1,1500:1520));
y1 = data{1,3}(data{1,6}(1,1500:1520));
x2 = size(imread(data{1,1}),2)+data{2,2}(data{1,6}(2,1500:1520));
y2 = data{2,3}(data{1,6}(2,1500:1520));

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