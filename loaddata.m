%% Load images for final assignment
% create list of images present for the structure from motion project
% Input: 
%   -none
%
% Output: 
%   -filelist: List of names of images to be processed in structure from motion
%   -imglist_gray : List of images in grayscale
%   -imglist_color: List of images in color
%
% Authors: 
%   -Bas Buller 4166566
%   -Rick Feith 4218272

%% list of images

function [names] = loaddata(folder);

% find all images in the folder
folder_names = dir(folder);
name_list = {folder_names(:).name};

names = {};
for i = 1:length(name_list)
    if (endsWith(name_list{i},".png"))
        names = [names; char(strcat(string(folder),"/",name_list{i}))];
    end
end

% add the imagenames to an array
% filelist = [];
%  
% for i = 1:length(names)
%      filelist = [filelist, strcat(string(folder),"/",names{i})];
% end
% add the images to an grayscale and color arrays
% imglist_gray = [];
% imglist_color = [];
% 
% for i = 1:length(filelist)
%     im = imread(char(filelist(i)));
%     imglist_gray = [[imglist_gray],[rgb2gray(im)]];
% %     imglist_color = [[imglist_color],(im)];
% end

end


