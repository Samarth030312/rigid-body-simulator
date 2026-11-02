function rigid_body_figures(outDir)
%RIGID_BODY_FIGURES  Regenerate every figure in docs/img from fresh simulations.
%
%   rigid_body_figures             writes to docs/img
%   rigid_body_figures(outDir)
%
%   Three model figures (scene at a representative instant beside its dashboard)
%   and one screenshot of the app. Nothing is drawn from stored data: each figure
%   is produced by running the model again, so a figure that no longer matches
%   the code cannot survive a regeneration.
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(root, 'matlab', 'setup_path.m'));
    if nargin < 1, outDir = fullfile(root, 'docs', 'img'); end
    if ~isfolder(outDir), mkdir(outDir); end

    cases = {
        'dice',    @() rbsim.dice.simulate(rbsim.dice.defaults()),    @rbsim.viz.diceScene,    @rbsim.viz.diceUpdate
        'robot',   @() rbsim.robot.simulate(pdParams()),              @rbsim.viz.robotScene,   @rbsim.viz.robotUpdate
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
        out = fullfile(outDir, [cases{i, 1} '.png']);
        exportgraphics(f, out, 'Resolution', 110);
        close(f);
        fprintf('wrote %s\n', out);
    end

    appShot(fullfile(outDir, 'app.png'));
end

function p = pdParams()
    p = rbsim.robot.defaults(); p.control.mode = 'pd'; p.tFinal = 3;
end

function appShot(out)
%   A screenshot of the app itself, mid-throw, so the front page shows the thing
%   a reader will actually open. Needs a display; skipped without one.
    if ~usejava('desktop') && isempty(getenv('DISPLAY')) && ~ispc && ~ismac
        fprintf('no display: skipped %s\n', out);
        return
    end
    app = RigidBodyLabApp();
    c = onCleanup(@() delete(app));
    app.run('dice');
    res = app.Results.dice;
    app.showSample('dice', round(0.42 * numel(res.t)));
    drawnow; pause(1.5);
    exportapp(app.UIFigure, out);
    fprintf('wrote %s\n', out);
end
