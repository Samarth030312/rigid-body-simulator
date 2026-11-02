function tests = test_end_to_end
%TEST_END_TO_END  The documented workflow works from a clean MATLAB path.
    tests = functiontests(localfunctions);
end

function root = repoRoot()
    root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
end

function testSetupPutsTheSimulatorOnACleanPath(tc)
%   README step 1: run setup from the repository root, on a default path.
    saved = path; c = onCleanup(@() path(saved));
    restoredefaultpath;
    run(fullfile(repoRoot(), 'matlab', 'setup_path.m'));
    for f = ["rbsim.math.rotBodyFixed", "rbsim.body.derivative", "rbsim.solver.rk4", ...
             "rbsim.dice.simulate", "rbsim.robot.simulate", "rbsim.fourbar.simulate", ...
             "rbsim.viz.theme", "rigid_body_lab"]
        tc.verifyNotEmpty(which(f), f + " is not on the path after setup_path");
    end
    r = rbsim.doctor('Quiet', true);
    tc.verifyEmpty(r(string({r.status}) == "FAIL"), 'rbsim.doctor reports a failing check');
end

function testExamplesReproduceTheirReferenceResults(tc)
%   README step 2: the worked examples run without the app and reproduce the
%   closed-form rotation matrices and physically consistent results.
    saved = path; c = onCleanup(@() path(saved));
    restoredefaultpath;
    addpath(fullfile(repoRoot(), 'matlab', 'scripts'));
    out = rigid_body_examples(Quiet=true);
    tc.verifyEqual(out.Rbody, [0.1830 0.6830 0.7071; -0.3245 0.7209 -0.6124; -0.9280 -0.1174 0.3536], ...
        'AbsTol', 5e-5, 'body-fixed X-Y-Z (60, 45, -75) deg');
    tc.verifyEqual(out.Rspace, [0.1830 0.6415 -0.7450; -0.6830 -0.4621 -0.5657; -0.7071 0.6124 0.3536], ...
        'AbsTol', 5e-5, 'space-fixed X-Y-Z (60, 45, -75) deg');
    tc.verifyTrue(out.dice.summary.settled, 'the thrown die did not come to rest');
    tc.verifyTrue(ismember(out.dice.summary.finalFace, 1:6));
    tc.verifyLessThan(max(abs(out.robotFree.diag.energy - out.robotFree.diag.energy(1))), 1e-4, ...
        'the undamped arm did not conserve energy');
    tc.verifyLessThan(out.robotMove.summary.trackingErrorMax, 1e-6);
    tc.verifyLessThan(out.fourbar.summary.maxClosureResidual, 1e-12);
end

function testTheAppOpensFromACleanPath(tc)
    saved = path; c = onCleanup(@() path(saved));
    restoredefaultpath;
    run(fullfile(repoRoot(), 'matlab', 'setup_path.m'));
    app = rigid_body_lab();
    c2 = onCleanup(@() delete(app));
    app.UIFigure.Visible = 'off';
    tc.verifyTrue(isvalid(app.UIFigure));
    tc.verifyEqual(numel(app.TabGroup.Children), 5, 'three models, validation and conventions');
    tc.verifyTrue(app.run('fourbar'));
end

function testThePlaygroundScriptRunsAsShipped(tc)
%   The script people edit must run unchanged, with its animation turned off.
    src = fileread(fullfile(repoRoot(), 'matlab', 'scripts', 'rigid_body_playground.m'));
    src = strrep(src, 'ANIMATE = true;', 'ANIMATE = false;');
    src = strrep(src, 'clear; close all;', '');
    src = strrep(src, "mfilename('fullpath')", ...
        "'" + string(fullfile(repoRoot(), 'matlab', 'scripts', 'rigid_body_playground')) + "'");
    tmp = [tempname '.m'];
    fid = fopen(tmp, 'w'); fwrite(fid, src); fclose(fid);
    c = onCleanup(@() delete(tmp));
    set(groot, 'DefaultFigureVisible', 'off');
    c2 = onCleanup(@() set(groot, 'DefaultFigureVisible', 'on'));
    [d, n] = fileparts(tmp); addpath(d); c3 = onCleanup(@() rmpath(d));
    evalc(n);
    tc.verifyGreaterThanOrEqual(numel(findall(groot, 'Type', 'figure')), 3, ...
        'the three worked models should each leave a figure');
    close(findall(groot, 'Type', 'figure'));
end

function testTheModelsDoNotDependOnTheAppOrTheGraphics(tc)
%   Layering: nothing in the mechanics packages may reach up into the app or the
%   drawing code, so every model runs headless and can be tested on its own.
    banned = {'uifigure', 'uiaxes', 'figure(', 'plot(', 'patch(', 'rbsim.viz', 'RigidBodyLabApp'};
    for pkg = {'+math', '+body', '+solver', '+dice', '+robot', '+fourbar'}
        files = dir(fullfile(repoRoot(), 'matlab', 'functions', '+rbsim', pkg{1}, '*.m'));
        for f = files'
            code = regexprep(fileread(fullfile(f.folder, f.name)), '%[^\n]*', '');
            for b = banned
                tc.verifyFalse(contains(code, b{1}), sprintf('%s uses %s', f.name, b{1}));
            end
        end
    end
end
