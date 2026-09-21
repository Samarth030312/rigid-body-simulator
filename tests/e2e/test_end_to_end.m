function tests = test_end_to_end
%TEST_END_TO_END Level 8: the documented workflow works from a clean path.
tests = functiontests(localfunctions);
end

function testExamplesRunFromCleanPath(tc)
% README step: on a fresh MATLAB path, running examples/runExamples reproduces
% the lecture reference matrices and physically consistent results.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
saved = path; cleanup = onCleanup(@() path(saved));
restoredefaultpath;
addpath(fullfile(root, 'examples'));
out = runExamples(Quiet=true);
verifyEqual(tc, out.Rbody, [0.1830 0.6830 0.7071; -0.3245 0.7209 -0.6124; -0.9280 -0.1174 0.3536], 'AbsTol', 5e-5);
verifyEqual(tc, out.Rspace, [0.1830 0.6415 -0.7450; -0.6830 -0.4621 -0.5657; -0.7071 0.6124 0.3536], 'AbsTol', 5e-5);
verifyTrue(tc, out.dice.summary.settled);
verifyTrue(tc, ismember(out.dice.summary.finalFace, 1:6));
verifyLessThan(tc, max(abs(out.robotFree.diag.energy - out.robotFree.diag.energy(1))), 1e-4);
verifyLessThan(tc, out.robotMove.summary.trackingErrorMax, 1e-6);
verifyLessThan(tc, out.fourbar.summary.maxClosureResidual, 1e-12);
end

function testLauncherOpensAppFromCleanPath(tc)
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
saved = path; cleanup = onCleanup(@() path(saved));
restoredefaultpath;
addpath(fullfile(root, 'app'));
app = launchRigidBodySimulator();
c2 = onCleanup(@() delete(app));
app.UIFigure.Visible = 'off';
verifyTrue(tc, isvalid(app.UIFigure));
verifyEqual(tc, numel(app.TabGroup.Children), 5);
verifyTrue(tc, app.run('fourbar'));
end

function testCoreDoesNotDependOnGuiOrViz(tc)
% Architecture rule: nothing in core/, dice/, robot/, fourbar/ calls the GUI or
% visualization layers, or creates graphics.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
banned = {'uifigure', 'uiaxes', 'figure(', 'plot(', 'patch(', 'viz', 'RigidBodySimulatorApp'};
for d = {'core', 'dice', 'robot', 'fourbar'}
    files = dir(fullfile(root, d{1}, '**', '*.m'));
    for f = files.'
        txt = fileread(fullfile(f.folder, f.name));
        code = regexprep(txt, '%[^\n]*', '');       % ignore comments
        for b = banned
            verifyFalse(tc, contains(code, b{1}), sprintf('%s uses %s', f.name, b{1}));
        end
    end
end
end

function testPlaygroundScriptRuns(tc)
% The tinkering script must run as shipped (without animation, figures hidden).
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
src = fileread(fullfile(root, 'examples', 'playground.m'));
src = strrep(src, 'ANIMATE = true;', 'ANIMATE = false;');
src = strrep(src, 'clear; close all;', '');
tmp = [tempname '.m'];
fid = fopen(tmp, 'w'); fwrite(fid, strrep(src, 'mfilename(''fullpath'')', ...
    ['''' fullfile(root, 'examples', 'playground') '''']));
fclose(fid);
c = onCleanup(@() delete(tmp));
set(groot, 'DefaultFigureVisible', 'off');
c2 = onCleanup(@() set(groot, 'DefaultFigureVisible', 'on'));
[d, n] = fileparts(tmp); addpath(d); c3 = onCleanup(@() rmpath(d));
evalc(n);
verifyGreaterThanOrEqual(tc, numel(findall(groot, 'Type', 'figure')), 3);
close(findall(groot, 'Type', 'figure'));
end
