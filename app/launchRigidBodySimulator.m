function launchRigidBodySimulator()
% launchRigidBodySimulator Configure and launch the ENR209 simulator.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root, 'core')));
addpath(genpath(fullfile(root, 'dice')));
addpath(genpath(fullfile(root, 'robot')));
addpath(genpath(fullfile(root, 'fourbar')));
addpath(genpath(fullfile(root, 'visualization')));
addpath(genpath(fullfile(root, 'config')));

fprintf('ENR209 Rigid Body Simulator\n');
fprintf('Root: %s\n', root);
fprintf('Physics engine: foundation stage\n');
fprintf('GUI: not yet implemented\n');
end
