function addProjectPaths()
%ADDPROJECTPATHS Add project source directories to the MATLAB path.
%   Adds core/, dice/, robot/, fourbar/, visualization/, app/ and config/
%   (with subfolders). Missing folders are skipped so the function works at
%   every development phase.
root = projectRoot();
dirs = {'core', 'dice', 'robot', 'fourbar', 'visualization', 'app', 'config'};
for i = 1:numel(dirs)
    d = fullfile(root, dirs{i});
    if isfolder(d)
        addpath(genpath(d));
    end
end
end
