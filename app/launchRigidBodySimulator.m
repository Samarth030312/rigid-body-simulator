function app = launchRigidBodySimulator()
%LAUNCHRIGIDBODYSIMULATOR Configure the MATLAB path and open the simulator GUI.
%
%   launchRigidBodySimulator()          opens the GUI
%   app = launchRigidBodySimulator();   also returns the app object
%
%   The mechanics can also be used without the GUI, e.g.
%     res = simulateDice(diceDefaultParams());
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'config'));
addProjectPaths();
fprintf('ENR209 Rigid Body Simulator (MATLAB %s)\nRoot: %s\n', version('-release'), root);
a = RigidBodySimulatorApp();
if nargout > 0, app = a; end
end
