function make_figures(outDir)
%MAKE_FIGURES Render reference figures of all three applications (headless-safe).
%
%   make_figures()             writes PNGs to docs/figures/
%   make_figures(outDir)
%
%   Each figure = final-state scene + dashboard, generated only from simulation
%   results (used as Level 6 visual evidence).
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
run(fullfile(root, 'matlab', 'setup_path.m'));
if nargin < 1, outDir = fullfile(root, 'docs', 'figures'); end
if ~isfolder(outDir), mkdir(outDir); end
cases = {'dice', @() rbsim.dice.simulate(rbsim.dice.defaults()), @rbsim.viz.diceScene, @rbsim.viz.diceUpdate;
         'robot', @() rbsim.robot.simulate(pdParams()), @rbsim.viz.robotScene, @rbsim.viz.robotUpdate;
         'fourbar', @() rbsim.fourbar.simulate(rbsim.fourbar.defaults()), @rbsim.viz.fourbarScene, @rbsim.viz.fourbarUpdate};
for i = 1:size(cases, 1)
    res = cases{i, 2}();
    f = figure('Visible', 'off', 'Position', [0 0 1400 700], 'Color', rbsim.viz.theme().surface);
    tl = tiledlayout(f, 2, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
    axS = nexttile(tl, 1, [2 2]);
    kStill = round(0.6 * numel(res.t));
    h = cases{i, 3}(axS, res);
    cases{i, 4}(h, kStill);
    axs = [nexttile(tl, 3), nexttile(tl, 4), nexttile(tl, 7), nexttile(tl, 8)];
    cur = rbsim.viz.dashboard(axs, res, cases{i, 1});
    rbsim.viz.setCursor(cur, res.t(kStill));
    exportgraphics(f, fullfile(outDir, [cases{i, 1} '.png']), 'Resolution', 110);
    close(f);
    fprintf('wrote %s\n', fullfile(outDir, [cases{i, 1} '.png']));
end
end

function p = pdParams()
p = rbsim.robot.defaults(); p.control.mode = 'pd'; p.tFinal = 3;
end
