function make_figures(outDir)
%MAKE_FIGURES Render reference figures of all three applications (headless-safe).
%
%   make_figures()             writes PNGs to docs/figures/
%   make_figures(outDir)
%
%   Each figure = final-state scene + dashboard, generated only from simulation
%   results (used as Level 6 visual evidence).
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'config')); addProjectPaths();
if nargin < 1, outDir = fullfile(root, 'docs', 'figures'); end
if ~isfolder(outDir), mkdir(outDir); end
cases = {'dice', @() simulateDice(diceDefaultParams()), @vizDiceScene, @vizDiceUpdate;
         'robot', @() simulateRobot(pdParams()), @vizRobotScene, @vizRobotUpdate;
         'fourbar', @() simulateFourbar(fourbarDefaultParams()), @vizFourbarScene, @vizFourbarUpdate};
for i = 1:size(cases, 1)
    res = cases{i, 2}();
    f = figure('Visible', 'off', 'Position', [0 0 1400 700], 'Color', vizTheme().surface);
    tl = tiledlayout(f, 2, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
    axS = nexttile(tl, 1, [2 2]);
    kStill = round(0.6 * numel(res.t));
    h = cases{i, 3}(axS, res);
    cases{i, 4}(h, kStill);
    axs = [nexttile(tl, 3), nexttile(tl, 4), nexttile(tl, 7), nexttile(tl, 8)];
    cur = vizDashboard(axs, res, cases{i, 1});
    vizSetCursor(cur, res.t(kStill));
    exportgraphics(f, fullfile(outDir, [cases{i, 1} '.png']), 'Resolution', 110);
    close(f);
    fprintf('wrote %s\n', fullfile(outDir, [cases{i, 1} '.png']));
end
end

function p = pdParams()
p = robotDefaultParams(); p.control.mode = 'pd'; p.tFinal = 3;
end
