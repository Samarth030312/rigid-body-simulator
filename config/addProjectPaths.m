function addProjectPaths()
% addProjectPaths Add project source directories to MATLAB path.
root = projectRoot();
addpath(genpath(fullfile(root, 'core')));
addpath(genpath(fullfile(root, 'dice')));
addpath(genpath(fullfile(root, 'robot')));
addpath(genpath(fullfile(root, 'fourbar')));
addpath(genpath(fullfile(root, 'visualization')));
addpath(genpath(fullfile(root, 'app')));
end
