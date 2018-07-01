% loaddata.m
%
% create list of images present for the structure from motion project
%
% Input: 
%   -folder: folder name containing the images 
%
% Output: 
%   -names: names of the images in the folder
%
% Authors: 
%   -Bas Buller 4166566
%   -Rick Feith 4218272

%% list of images

function [names] = loaddata(folder)

% find all images in the folder
folder_names = dir(folder);
name_list = {folder_names(:).name};

% return names as a single struct
names = {};
for i = 1:length(name_list)
    if (endsWith(name_list{i},".png"))
        names = [names; char(strcat(string(folder),"/",name_list{i}))];
    end
end

end


